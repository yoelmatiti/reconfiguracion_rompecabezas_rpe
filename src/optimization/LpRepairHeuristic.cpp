#include "optimization/LpRepairHeuristic.h"
#include "domain/Validator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace optimization {

namespace {
    const double EPSILON = 1e-6;
    const double TIME_EPS = 1e-9;
}

// ============================================================================
// RepairResult
// ============================================================================

RepairResult::RepairResult()
    : success(false), cost(0.0), unassigned(), errorMessage() {}

RepairResult::RepairResult(bool success,
                           std::unordered_map<int, int> solution,
                           double cost)
    : success(success),
      solution(std::move(solution)),
      cost(cost),
      unassigned(),
      errorMessage() {}

// ============================================================================
// Config
// ============================================================================

LpRepairHeuristic::Config::Config()
    : epsilon(EPSILON),
      maxIterations(100),
      timeLimitMs(1000.0) {}

LpRepairHeuristic::Config::Config(const SolverConfig& solverCfg)
    : epsilon(solverCfg.primalTolerance > 0.0 ? solverCfg.primalTolerance : EPSILON),
      maxIterations(solverCfg.maxNodes > 0 ? solverCfg.maxNodes : 100),
      timeLimitMs(solverCfg.maxTimeSeconds > 0.0 ? solverCfg.maxTimeSeconds * 1000.0 : 1000.0) {}

// ============================================================================
// RepairState
// ============================================================================

LpRepairHeuristic::RepairState::RepairState(int numPieces_, int numCells_)
    : cellToPiece(numCells_, -1),
      numPieces(numPieces_),
      numCells(numCells_) {
    assignment.reserve(numPieces);
}

// ============================================================================
// Punto de entrada principal
// ============================================================================

RepairResult LpRepairHeuristic::repair(
    const domain::Snapshot& snapshot,
    const std::vector<double>& lpSolution,
    const std::vector<int>& fixedToOne,
    const std::vector<int>& fixedToZero,
    const ClpRelaxationSolver& clpSolver,
    const Config& config) {

    const int numPieces = static_cast<int>(snapshot.getPieces().size());
    const int numCells = static_cast<int>(snapshot.getCells().size());

    if (numPieces <= 0 || numCells <= 0) {
        return RepairResult(false, {}, 0.0);
    }

    RepairState state(numPieces, numCells);

    // Preparar sets de fijaciones para lookup O(1)
    state.fixedToOneSet.assign(
        clpSolver.getNumColumns() > 0 ? clpSolver.getNumColumns() : numPieces * numCells,
        false
    );
    state.fixedToZeroSet.assign(state.fixedToOneSet.size(), false);

    for (int colIdx : fixedToOne) {
        if (colIdx >= 0 && colIdx < static_cast<int>(state.fixedToOneSet.size())) {
            state.fixedToOneSet[colIdx] = true;
        }
    }
    for (int colIdx : fixedToZero) {
        if (colIdx >= 0 && colIdx < static_cast<int>(state.fixedToZeroSet.size())) {
            state.fixedToZeroSet[colIdx] = true;
        }
    }

    std::vector<int> pendingPieces;
    pendingPieces.reserve(numPieces);

    // ------------------------------------------------------------------------
    // Fase 1: Redondeo tentativo
    // ------------------------------------------------------------------------
    if (!tentativeRounding(snapshot, lpSolution, clpSolver, state, pendingPieces)) {
        return RepairResult(false, {}, 0.0);
    }

    // ------------------------------------------------------------------------
    // Fase 2: Truncación selectiva
    // ------------------------------------------------------------------------
    if (!selectiveTruncation(snapshot, lpSolution, clpSolver, state,
                             pendingPieces, config.timeLimitMs)) {
        return RepairResult(false, {}, 0.0);
    }

    // ------------------------------------------------------------------------
    // Fase 3: Reinserción factible
    // ------------------------------------------------------------------------
    if (!feasibleInsertion(snapshot, lpSolution, clpSolver, state,
                           pendingPieces, config.timeLimitMs)) {
        return RepairResult(false, {}, 0.0);
    }

    // ------------------------------------------------------------------------
    // Fase 4: Verificación final
    // ------------------------------------------------------------------------
    if (static_cast<int>(state.assignment.size()) != numPieces) {
        return RepairResult(false, {}, 0.0);
    }

    auto validation = domain::Validator::validate(state.assignment, snapshot);
    if (!validation.first) {
        return RepairResult(false, {}, 0.0);
    }

    return RepairResult(true, std::move(state.assignment), validation.second);
}

// ============================================================================
// Fase 1: Redondeo tentativo
// ============================================================================

bool LpRepairHeuristic::tentativeRounding(
    const domain::Snapshot& snapshot,
    const std::vector<double>& lpSolution,
    const ClpRelaxationSolver& clpSolver,
    RepairState& state,
    std::vector<int>& pendingPieces) {

    const auto& pieces = snapshot.getPieces();
    const int numPieces = state.numPieces;
    const int numCells = state.numCells;

    // 1. Aplicar fijaciones a 1 (B&B)
    for (int colIdx = 0; colIdx < static_cast<int>(state.fixedToOneSet.size()); ++colIdx) {
        if (!state.fixedToOneSet[colIdx]) continue;

        const auto& meta = clpSolver.getColumnMeta(colIdx);

        if (state.isCellOccupied(meta.cellId)) {
            return false;  // Conflicto: dos fijaciones a la misma celda
        }

        state.assignment[meta.pieceId] = meta.cellId;
        state.cellToPiece[meta.cellId] = meta.pieceId;
    }

    // 2. Para cada pieza no asignada, elegir la celda con mayor x_ij
    //    Criterio lexicográfico: mayor LP, menor costo, menor índice
    for (int i = 0; i < numPieces; ++i) {
        const int pieceId = pieces[i].getId();
        if (state.assignment.find(pieceId) != state.assignment.end()) continue;

        int bestCell = -1;
        double bestLpValue = -1.0;
        double bestCost = std::numeric_limits<double>::max();

        for (int j = 0; j < numCells; ++j) {
            if (state.isCellOccupied(j)) continue;

            const int colIdx = clpSolver.getColumnIndex(pieceId, j);
            if (colIdx < 0) continue;
            if (colIdx < static_cast<int>(state.fixedToZeroSet.size()) &&
                state.fixedToZeroSet[colIdx]) continue;

            const double lpValue = lpSolution[colIdx];
            const double moveCost = snapshot.getMoveCost(pieceId, j);

            bool isBetter = false;
            if (bestCell < 0) {
                isBetter = true;
            } else if (lpValue > bestLpValue + EPSILON) {
                isBetter = true;
            } else if (std::abs(lpValue - bestLpValue) <= EPSILON) {
                if (moveCost < bestCost - EPSILON) {
                    isBetter = true;
                } else if (std::abs(moveCost - bestCost) <= EPSILON && j < bestCell) {
                    isBetter = true;
                }
            }

            if (isBetter) {
                bestCell = j;
                bestLpValue = lpValue;
                bestCost = moveCost;
            }
        }

        if (bestCell >= 0) {
            state.assignment[pieceId] = bestCell;
            state.cellToPiece[bestCell] = pieceId;
        } else {
            pendingPieces.push_back(pieceId);
        }
    }

    return true;
}

// ============================================================================
// Fase 2: Truncación selectiva (INCREMENTAL)
// ============================================================================

bool LpRepairHeuristic::selectiveTruncation(
    const domain::Snapshot& snapshot,
    const std::vector<double>& lpSolution,
    const ClpRelaxationSolver& clpSolver,
    RepairState& state,
    std::vector<int>& pendingPieces,
    double timeLimitMs) {

    auto startTime = std::chrono::steady_clock::now();

    // Buffer reutilizable de violadores
    std::vector<int> violatingPieces;
    violatingPieces.reserve(state.numPieces);

    while (true) {
        // --------------------------------------------------------------------
        // Verificar tiempo
        // --------------------------------------------------------------------
        auto currentTime = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double, std::milli>(
            currentTime - startTime).count();
        if (elapsed > timeLimitMs) {
            return false;
        }

        // --------------------------------------------------------------------
        // Detectar violaciones locales (solo celdas vecinas)
        // --------------------------------------------------------------------
        violatingPieces.clear();
        for (const auto& entry : state.assignment) {
            const int pieceId = entry.first;
            const int cellId = entry.second;

            if (hasLocalViolation(pieceId, cellId, state, snapshot)) {
                violatingPieces.push_back(pieceId);
            }
        }

        if (violatingPieces.empty()) {
            return true;  // No hay violaciones
        }

        // --------------------------------------------------------------------
        // Filtrar violadores no fijados por B&B
        // --------------------------------------------------------------------
        std::vector<int> unfixedViolators;
        unfixedViolators.reserve(violatingPieces.size());

        for (int pieceId : violatingPieces) {
            auto it = state.assignment.find(pieceId);
            if (it == state.assignment.end()) continue;

            const int colIdx = clpSolver.getColumnIndex(pieceId, it->second);
            if (colIdx < 0) continue;
            if (colIdx < static_cast<int>(state.fixedToOneSet.size()) &&
                state.fixedToOneSet[colIdx]) {
                continue;  // Fijado por B&B: no podemos truncar
            }
            unfixedViolators.push_back(pieceId);
        }

        if (unfixedViolators.empty()) {
            return false;  // Todas las violaciones son de piezas fijadas
        }

        // --------------------------------------------------------------------
        // Encontrar el peor violador y truncarlo
        // --------------------------------------------------------------------
        const int worstPiece = findWorstViolator(
            unfixedViolators, state, lpSolution, snapshot, clpSolver
        );

        if (worstPiece < 0) {
            return false;
        }

        // Truncar
        auto it = state.assignment.find(worstPiece);
        if (it != state.assignment.end()) {
            const int oldCell = it->second;
            state.cellToPiece[oldCell] = -1;
            state.assignment.erase(it);
            pendingPieces.push_back(worstPiece);
        }
    }
}

// ============================================================================
// Fase 3: Reinserción factible (First-Fail)
// ============================================================================

bool LpRepairHeuristic::feasibleInsertion(
    const domain::Snapshot& snapshot,
    const std::vector<double>& lpSolution,
    const ClpRelaxationSolver& clpSolver,
    RepairState& state,
    std::vector<int>& pendingPieces,
    double timeLimitMs) {

    auto startTime = std::chrono::steady_clock::now();

    // Buffer reutilizable de celdas factibles
    std::vector<int> feasibleCells;
    feasibleCells.reserve(state.numCells);

    while (!pendingPieces.empty()) {
        // --------------------------------------------------------------------
        // Verificar tiempo
        // --------------------------------------------------------------------
        auto currentTime = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double, std::milli>(
            currentTime - startTime).count();
        if (elapsed > timeLimitMs) {
            return false;
        }

        // --------------------------------------------------------------------
        // Seleccionar la pieza pendiente con MENOS opciones factibles
        // (First-Fail Principle)
        // --------------------------------------------------------------------
        int selectedPiece = -1;
        int selectedCell = -1;
        size_t minFeasibleCount = std::numeric_limits<size_t>::max();

        for (int pieceId : pendingPieces) {
            feasibleCells.clear();
            getFeasibleCells(pieceId, state, snapshot, clpSolver, feasibleCells);

            if (feasibleCells.empty()) {
                return false;  // Pieza sin opciones: reparación falla
            }

            const size_t count = feasibleCells.size();

            // Criterio de selección de pieza: menor |F(i)|, empate por mayor
            // costo de pieza, empate final por menor id.
            bool isBetter = false;
            if (count < minFeasibleCount) {
                isBetter = true;
            } else if (count == minFeasibleCount) {
                const double pieceCost = snapshot.getPieces()[pieceId].getCost();
                if (selectedPiece < 0) {
                    isBetter = true;
                } else {
                    const double selectedPieceCost =
                        snapshot.getPieces()[selectedPiece].getCost();
                    if (pieceCost > selectedPieceCost + EPSILON) {
                        isBetter = true;
                    } else if (std::abs(pieceCost - selectedPieceCost) <= EPSILON &&
                               pieceId < selectedPiece) {
                        isBetter = true;
                    }
                }
            }

            if (isBetter) {
                // Elegir la mejor celda para esta pieza
                int bestCell = -1;
                double bestCellCost = std::numeric_limits<double>::max();
                double bestCellLp = -1.0;

                for (int cellId : feasibleCells) {
                    const int colIdx = clpSolver.getColumnIndex(pieceId, cellId);
                    const double lpVal =
                        (colIdx >= 0 && colIdx < static_cast<int>(lpSolution.size()))
                            ? lpSolution[colIdx]
                            : 0.0;
                    const double moveCost = snapshot.getMoveCost(pieceId, cellId);

                    bool isBetterCell = false;
                    if (bestCell < 0) {
                        isBetterCell = true;
                    } else if (moveCost < bestCellCost - EPSILON) {
                        isBetterCell = true;
                    } else if (std::abs(moveCost - bestCellCost) <= EPSILON) {
                        if (lpVal > bestCellLp + EPSILON) {
                            isBetterCell = true;
                        } else if (std::abs(lpVal - bestCellLp) <= EPSILON &&
                                   cellId < bestCell) {
                            isBetterCell = true;
                        }
                    }

                    if (isBetterCell) {
                        bestCell = cellId;
                        bestCellCost = moveCost;
                        bestCellLp = lpVal;
                    }
                }

                selectedPiece = pieceId;
                selectedCell = bestCell;
                minFeasibleCount = count;
            }
        }

        if (selectedPiece < 0 || selectedCell < 0) {
            return false;
        }

        // --------------------------------------------------------------------
        // Asignar la pieza seleccionada
        // --------------------------------------------------------------------
        state.assignment[selectedPiece] = selectedCell;
        state.cellToPiece[selectedCell] = selectedPiece;

        // Eliminar de pendingPieces (swap-and-pop)
        auto pit = std::find(pendingPieces.begin(), pendingPieces.end(), selectedPiece);
        if (pit != pendingPieces.end()) {
            *pit = pendingPieces.back();
            pendingPieces.pop_back();
        }
    }

    return true;
}

// ============================================================================
// checkNeighborMatching — O(1) por vecino
// ============================================================================

bool LpRepairHeuristic::checkNeighborMatching(
    int pieceId,
    int cellId,
    const RepairState& state,
    const domain::Snapshot& snapshot) {

    const auto& pieces = snapshot.getPieces();
    const auto& geometry = snapshot.getGeometry();

    if (pieceId < 0 || pieceId >= static_cast<int>(pieces.size())) return false;
    if (cellId < 0 || cellId >= state.numCells) return false;

    const auto& piece = pieces[pieceId];
    const auto adjacentCells = geometry.getAdjacentCells(cellId);

    for (int i = 0; i < adjacentCells.count; ++i) {
        const int neighborCellId = adjacentCells.cells[i].first;
        const int neighborPieceId = state.pieceAt(neighborCellId);  // ← O(1)

        if (neighborPieceId < 0) continue;  // Celda vacía

        const auto& neighborPiece = pieces[neighborPieceId];
        const domain::Side sideFromThis = adjacentCells.cells[i].second;
        const domain::Side sideFromNeighbor = domain::getOppositeSide(sideFromThis);

        if (piece.getColor(sideFromThis) != neighborPiece.getColor(sideFromNeighbor)) {
            return false;
        }
    }

    return true;
}

// ============================================================================
// hasLocalViolation — verificación local de una asignación
// ============================================================================

bool LpRepairHeuristic::hasLocalViolation(
    int pieceId,
    int cellId,
    const RepairState& state,
    const domain::Snapshot& snapshot) {

    // Verificar solo desajustes de color con vecinos
    return !checkNeighborMatching(pieceId, cellId, state, snapshot);
}

// ============================================================================
// getFeasibleCells — usa cellToPiece O(1)
// ============================================================================

void LpRepairHeuristic::getFeasibleCells(
    int pieceId,
    const RepairState& state,
    const domain::Snapshot& snapshot,
    const ClpRelaxationSolver& clpSolver,
    std::vector<int>& outCells) {

    outCells.clear();

    const auto& pieces = snapshot.getPieces();
    if (pieceId < 0 || pieceId >= static_cast<int>(pieces.size())) return;

    for (int cellId = 0; cellId < state.numCells; ++cellId) {
        // 1. Celda libre
        if (state.isCellOccupied(cellId)) continue;

        // 2. Variable existe
        const int colIdx = clpSolver.getColumnIndex(pieceId, cellId);
        if (colIdx < 0) continue;

        // 3. No fijada a 0
        if (colIdx < static_cast<int>(state.fixedToZeroSet.size()) &&
            state.fixedToZeroSet[colIdx]) continue;

        // 4. Compatibilidad de vecinos (con cellToPiece O(1))
        if (!checkNeighborMatching(pieceId, cellId, state, snapshot)) continue;

        outCells.push_back(cellId);
    }
}

// ============================================================================
// findWorstViolator
// ============================================================================

int LpRepairHeuristic::findWorstViolator(
    const std::vector<int>& violatingPieces,
    const RepairState& state,
    const std::vector<double>& lpSolution,
    const domain::Snapshot& snapshot,
    const ClpRelaxationSolver& clpSolver) {

    int worstPiece = -1;
    double worstLpValue = std::numeric_limits<double>::max();
    double worstCost = -1.0;

    for (int pieceId : violatingPieces) {
        auto it = state.assignment.find(pieceId);
        if (it == state.assignment.end()) continue;

        const int cellId = it->second;
        const int colIdx = clpSolver.getColumnIndex(pieceId, cellId);
        const double lpVal = (colIdx >= 0 && colIdx < static_cast<int>(lpSolution.size()))
                              ? lpSolution[colIdx] : 0.0;
        const double moveCost = snapshot.getMoveCost(pieceId, cellId);

        // Peor = menor respaldo LP; empate: mayor costo
        bool isWorse = false;
        if (worstPiece < 0) {
            isWorse = true;
        } else if (lpVal < worstLpValue - EPSILON) {
            isWorse = true;
        } else if (std::abs(lpVal - worstLpValue) <= EPSILON) {
            if (moveCost > worstCost + EPSILON) {
                isWorse = true;
            } else if (std::abs(moveCost - worstCost) <= EPSILON && pieceId < worstPiece) {
                isWorse = true;
            }
        }

        if (isWorse) {
            worstPiece = pieceId;
            worstLpValue = lpVal;
            worstCost = moveCost;
        }
    }

    return worstPiece;
}

} // namespace optimization