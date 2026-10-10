#include "optimization/BranchAndBoundA0.h"
#include "domain/Validator.h"
#include "optimization/HeuristicaH1.h"
// NOTA: H2 NO se incluye (según el enunciado, A0 solo usa H1 como warm start)

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace optimization {

// ============================================================================
// Constructor
// ============================================================================

BranchAndBoundA0::BranchAndBoundA0(const domain::Snapshot& snapshot,
                                   const A0Config& config)
    : snapshot_(snapshot),
      config_(config),
      startTime_(std::chrono::steady_clock::now()) {

    try {
        clpSolver_ = std::unique_ptr<ClpRelaxationSolver>(
            new ClpRelaxationSolver(snapshot)
        );
    } catch (const std::exception& e) {
        throw std::runtime_error(
            std::string("Error al construir el modelo raíz CLP: ") + e.what()
        );
    }

    const auto& varMap = clpSolver_->getVarMap();
    columnToPair_.reserve(varMap.size());
    for (const auto& pair : varMap) {
        int key = pair.first;
        int colIdx = pair.second;
        int pieceId = key / 1000000;
        int cellId = key % 1000000;
        columnToPair_[colIdx] = std::make_pair(pieceId, cellId);
    }
}

// ============================================================================
// solve() — algoritmo principal (Listing 3)
// ============================================================================

A0Result BranchAndBoundA0::solve() {
    A0Result result;
    auto startTime = std::chrono::steady_clock::now();

    try {
        // ---------------------------------------------------------------------
        // PASO 1: Warm start con H1 (SOLO H1, según el enunciado)
        // ---------------------------------------------------------------------
        if (config_.useWarmStartH1) {
            try {
                Solution h1Solution = HeuristicH1::solve(
                    snapshot_, Solution(), config_.maxTimeSeconds
                );

                std::unordered_map<int, int> h1Map;
                h1Map.reserve(h1Solution.size());
                for (size_t i = 0; i < h1Solution.size(); ++i) {
                    if (h1Solution[i] >= 0) {
                        h1Map[static_cast<int>(i)] = h1Solution[i];
                    }
                }

                auto val = domain::Validator::validate(h1Map, snapshot_);
                if (val.first) {
                    updateIncumbent(h1Map, val.second, result);
                }
            } catch (const std::exception& e) {
                std::cerr << "[A0] Advertencia: H1 warm start falló: "
                          << e.what() << std::endl;
                // El warm start es opcional; A0 puede continuar sin él.
            }
        }

        // ---------------------------------------------------------------------
        // PASO 2: Resolver la relajación raíz
        // ---------------------------------------------------------------------
        std::vector<int> emptyFixedTo1;
        std::vector<int> emptyFixedTo0;
        double remainingTime = getRemainingTime(config_.maxTimeSeconds);

        LPSolverResult rootLP = clpSolver_->solveRelaxation(
            emptyFixedTo1, emptyFixedTo0, remainingTime
        );
        result.lpCalls++;
        result.simplexIterations += rootLP.iterations;

        // ---------------------------------------------------------------------
        // PASO 3: Clasificar estado de la raíz
        // ---------------------------------------------------------------------
        if (rootLP.status == LPSolverStatus::INFEASIBLE) {
            result.status = SolverStatus::INFEASIBLE;
            result.statusMessage = "Raíz infactible";
            result.executionTimeSeconds = elapsedSeconds(startTime);
            return result;
        }

        if (rootLP.status == LPSolverStatus::LIMIT) {
            result.lowerBound = 0.0;  // Cota válida débil (cij >= 0)
            if (!result.hasIncumbent()) {
                result.status = SolverStatus::NO_INCUMBENT;
                result.statusMessage = "Tiempo agotado en raíz sin incumbente";
            } else {
                result.status = SolverStatus::FEASIBLE;
                result.gap = computeGap(result.upperBound, result.lowerBound);
                result.statusMessage = "Tiempo agotado en raíz con incumbente";
            }
            result.executionTimeSeconds = elapsedSeconds(startTime);
            return result;
        }

        if (rootLP.status == LPSolverStatus::SOLVER_ERROR ||
            rootLP.status == LPSolverStatus::ABANDONED) {
            result.status = SolverStatus::SOLVER_ERROR;
            result.statusMessage = "Error al resolver la raíz LP";
            result.executionTimeSeconds = elapsedSeconds(startTime);
            return result;
        }

        // ---------------------------------------------------------------------
        // PASO 4: Crear nodo raíz y agregar a OPEN
        // ---------------------------------------------------------------------
        BBNode rootNode(0, 0, rootLP.objective);
        rootNode.primal = std::move(rootLP.primal);

        std::priority_queue<BBNode, std::vector<BBNode>, CompareBestBound> open;
        open.push(std::move(rootNode));
        result.maxQueueSize = 1;

        // ---------------------------------------------------------------------
        // PASO 5: Bucle principal de Branch-and-Bound
        // ---------------------------------------------------------------------
        int nodeCounter = 1;
        bool timeExpired = false;
        bool solverError = false;

        while (!open.empty()) {
            double elapsed = elapsedSeconds(startTime);

            if (elapsed >= config_.maxTimeSeconds) {
                timeExpired = true;
                break;
            }

            BBNode node = open.top();
            open.pop();
            result.nodesExplored++;

            // Poda por cota (a nivel de bucle principal)
            if (node.lowerBound >= result.upperBound - config_.epsilonBound) {
                result.prunedByBound++;
                continue;
            }

            double remaining = config_.maxTimeSeconds - elapsed;
            bool processed = processNode(node, open, result, nodeCounter,
                                         remaining, solverError);

            if (solverError) {
                result.status = SolverStatus::SOLVER_ERROR;
                result.statusMessage = "Error del solver durante la expansión";
                result.executionTimeSeconds = elapsedSeconds(startTime);
                return result;
            }

            if (!processed) {
                timeExpired = true;
                break;
            }

            if (static_cast<int>(open.size()) > result.maxQueueSize) {
                result.maxQueueSize = static_cast<int>(open.size());
            }
        }

        // ---------------------------------------------------------------------
        // PASO 6: Clasificación del estado final (Listing 3, líneas 85-99)
        // ---------------------------------------------------------------------
        result.executionTimeSeconds = elapsedSeconds(startTime);

        if (!timeExpired && open.empty()) {
            // Búsqueda completa
            if (!result.hasIncumbent()) {
                result.status = SolverStatus::INFEASIBLE;
                result.statusMessage = "Árbol explorado sin solución factible";
            } else {
                result.status = SolverStatus::OPTIMAL;
                result.lowerBound = result.upperBound;
                result.gap = 0.0;
                result.statusMessage = "Optimalidad demostrada";
            }
        } else {
            // Búsqueda incompleta por tiempo
            if (result.hasIncumbent()) {
                result.status = SolverStatus::FEASIBLE;
                // LB global = min LB de nodos abiertos
                result.lowerBound = open.empty()
                    ? result.upperBound
                    : open.top().lowerBound;
                result.gap = computeGap(result.upperBound, result.lowerBound);
                result.statusMessage = "Búsqueda detenida por tiempo (con incumbente)";
            } else {
                result.status = SolverStatus::NO_INCUMBENT;
                result.lowerBound = open.empty() ? 0.0 : open.top().lowerBound;
                result.statusMessage = "Búsqueda detenida por tiempo (sin incumbente)";
            }
        }

    } catch (const std::exception& e) {
        result.status = SolverStatus::SOLVER_ERROR;
        result.statusMessage = std::string("Excepción en A0: ") + e.what();
        result.executionTimeSeconds = elapsedSeconds(startTime);
    }

    return result;
}

// ============================================================================
// processNode — expansión de un nodo
// ============================================================================

bool BranchAndBoundA0::processNode(
    BBNode& node,
    std::priority_queue<BBNode, std::vector<BBNode>, CompareBestBound>& open,
    A0Result& result,
    int& nodeCounter,
    double timeLimit,
    bool& solverError) {

    // -------------------------------------------------------------------------
    // 1. Verificar si la solución LP es entera
    // -------------------------------------------------------------------------
    if (isIntegerSolution(node.primal, config_.epsilonInt)) {
        auto integerSol = convertToIntegerSolution(node.primal);
        auto val = domain::Validator::validate(integerSol, snapshot_);
        if (val.first) {
            updateIncumbent(integerSol, val.second, result);
        }
        return true;  // Nodo procesado (no ramificar)
    }

    // -------------------------------------------------------------------------
    // 2. Intentar heurística de reparación (opcional)
    // -------------------------------------------------------------------------
    if (config_.useRepairHeuristic) {
        tryRepairHeuristic(node, result, timeLimit * 1000.0);
    }

    // -------------------------------------------------------------------------
    // 3. Verificar tiempo antes de ramificar
    // -------------------------------------------------------------------------
    if (timeLimit <= 0.01) {
        open.push(std::move(node));
        return false;  // Tiempo agotado
    }

    // -------------------------------------------------------------------------
    // 4. Seleccionar variable de ramificación
    // -------------------------------------------------------------------------
    int branchCol = selectBranchVariable(node.primal);

    if (branchCol < 0) {
        // No hay variable fraccionaria: tratar como entera
        auto integerSol = convertToIntegerSolution(node.primal);
        auto val = domain::Validator::validate(integerSol, snapshot_);
        if (val.first) {
            updateIncumbent(integerSol, val.second, result);
        }
        return true;
    }

    // -------------------------------------------------------------------------
    // 5. Crear nodos hijos (reutilizar fijaciones del padre)
    // -------------------------------------------------------------------------
    BBNode child0(++nodeCounter, node.depth + 1, node.lowerBound);
    BBNode child1(++nodeCounter, node.depth + 1, node.lowerBound);

    // Copiar fijaciones y agregar la nueva
    child0.fixedToOne = node.fixedToOne;
    child0.fixedToZero = node.fixedToZero;
    child0.fixedToZero.push_back(branchCol);  // Hijo 0: x_ij = 0

    child1.fixedToOne = node.fixedToOne;
    child1.fixedToZero = node.fixedToZero;
    child1.fixedToOne.push_back(branchCol);   // Hijo 1: x_ij = 1

    // -------------------------------------------------------------------------
    // 6. Evaluar los dos hijos (con protección de tiempo)
    // -------------------------------------------------------------------------
    std::vector<BBNode> candidates;
    candidates.reserve(2);

    // --- Hijo 0 (x = 0) ---
    {
        double remaining = getRemainingTime(config_.maxTimeSeconds);
        if (remaining <= 0.01) {
            open.push(std::move(node));
            return false;
        }

        LPSolverResult lp0 = clpSolver_->solveRelaxation(
            child0.fixedToOne, child0.fixedToZero, remaining
        );
        result.lpCalls++;
        result.simplexIterations += lp0.iterations;

        if (lp0.status == LPSolverStatus::INFEASIBLE) {
            result.prunedByInfeasibility++;
        } else if (lp0.status == LPSolverStatus::OPTIMAL) {
            child0.lowerBound = lp0.objective;
            child0.primal = std::move(lp0.primal);

            if (child0.lowerBound < result.upperBound - config_.epsilonBound) {
                candidates.push_back(std::move(child0));
            } else {
                result.prunedByBound++;
            }
        } else if (lp0.status == LPSolverStatus::LIMIT) {
            open.push(std::move(node));
            return false;
        } else {
            // SOLVER_ERROR o ABANDONED
            solverError = true;
            return false;
        }
    }

    // --- Hijo 1 (x = 1) ---
    {
        double remaining = getRemainingTime(config_.maxTimeSeconds);
        if (remaining <= 0.01) {
            for (auto& c : candidates) open.push(std::move(c));
            open.push(std::move(node));
            return false;
        }

        LPSolverResult lp1 = clpSolver_->solveRelaxation(
            child1.fixedToOne, child1.fixedToZero, remaining
        );
        result.lpCalls++;
        result.simplexIterations += lp1.iterations;

        if (lp1.status == LPSolverStatus::INFEASIBLE) {
            result.prunedByInfeasibility++;
        } else if (lp1.status == LPSolverStatus::OPTIMAL) {
            child1.lowerBound = lp1.objective;
            child1.primal = std::move(lp1.primal);

            if (child1.lowerBound < result.upperBound - config_.epsilonBound) {
                candidates.push_back(std::move(child1));
            } else {
                result.prunedByBound++;
            }
        } else if (lp1.status == LPSolverStatus::LIMIT) {
            for (auto& c : candidates) open.push(std::move(c));
            open.push(std::move(node));
            return false;
        } else {
            solverError = true;
            return false;
        }
    }

    // -------------------------------------------------------------------------
    // 7. Insertar candidatos supervivientes en OPEN
    // -------------------------------------------------------------------------
    for (auto& candidate : candidates) {
        open.push(std::move(candidate));
    }

    return true;
}

// ============================================================================
// isIntegerSolution
// ============================================================================

bool BranchAndBoundA0::isIntegerSolution(const std::vector<double>& primal,
                                          double epsilonInt) const {
    for (double val : primal) {
        if (val > epsilonInt && val < 1.0 - epsilonInt) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// convertToIntegerSolution
// ============================================================================

std::unordered_map<int, int> BranchAndBoundA0::convertToIntegerSolution(
    const std::vector<double>& primal) const {

    std::unordered_map<int, int> solution;
    solution.reserve(primal.size());

    for (size_t colIdx = 0; colIdx < primal.size(); ++colIdx) {
        if (primal[colIdx] > 0.5) {
            auto it = columnToPair_.find(static_cast<int>(colIdx));
            if (it != columnToPair_.end()) {
                solution[it->second.first] = it->second.second;
            }
        }
    }
    return solution;
}

// ============================================================================
// selectBranchVariable — variable más fraccionaria
// ============================================================================

int BranchAndBoundA0::selectBranchVariable(const std::vector<double>& primal) const {
    int bestCol = -1;
    double bestFractionality = std::numeric_limits<double>::max();
    int bestPieceId = std::numeric_limits<int>::max();
    int bestCellId = std::numeric_limits<int>::max();

    for (size_t colIdx = 0; colIdx < primal.size(); ++colIdx) {
        double val = primal[colIdx];
        if (val <= config_.epsilonInt || val >= 1.0 - config_.epsilonInt) {
            continue;
        }

        double fractionality = std::abs(val - 0.5);
        auto it = columnToPair_.find(static_cast<int>(colIdx));
        if (it == columnToPair_.end()) continue;

        int pieceId = it->second.first;
        int cellId = it->second.second;

        bool isBetter = false;
        if (bestCol < 0) {
            isBetter = true;
        } else if (fractionality < bestFractionality - 1e-12) {
            isBetter = true;
        } else if (std::abs(fractionality - bestFractionality) <= 1e-12) {
            if (pieceId < bestPieceId ||
                (pieceId == bestPieceId && cellId < bestCellId)) {
                isBetter = true;
            }
        }

        if (isBetter) {
            bestCol = static_cast<int>(colIdx);
            bestFractionality = fractionality;
            bestPieceId = pieceId;
            bestCellId = cellId;
        }
    }
    return bestCol;
}

// ============================================================================
// updateIncumbent — con desempate lexicográfico
// ============================================================================

bool BranchAndBoundA0::updateIncumbent(
    const std::unordered_map<int, int>& candidate,
    double cost,
    A0Result& result) {

    const double EPS = config_.epsilonBound;

    // Caso 1: mejor costo
    if (cost < result.upperBound - EPS) {
        result.incumbent = candidate;
        result.upperBound = cost;
        result.integerSolutionsFound++;
        return true;
    }

    // Caso 2: empate dentro de tolerancia → desempatar lexicográficamente
    if (std::abs(cost - result.upperBound) <= EPS) {
        // Comparar por orden lexicográfico: pieceId 0, 1, 2, ...
        const auto& pieces = snapshot_.getPieces();
        const size_t numPieces = pieces.size();

        bool isLexSmaller = false;
        for (size_t i = 0; i < numPieces; ++i) {
            int pieceId = pieces[i].getId();

            auto newIt = candidate.find(pieceId);
            auto oldIt = result.incumbent.find(pieceId);

            int newCell = (newIt != candidate.end()) ? newIt->second : -1;
            int oldCell = (oldIt != result.incumbent.end()) ? oldIt->second : -1;

            if (newCell < oldCell) {
                isLexSmaller = true;
                break;
            } else if (newCell > oldCell) {
                break;
            }
        }

        if (isLexSmaller) {
            result.incumbent = candidate;
            result.integerSolutionsFound++;
            return true;
        }
    }

    return false;
}

// ============================================================================
// tryRepairHeuristic
// ============================================================================

bool BranchAndBoundA0::tryRepairHeuristic(const BBNode& node,
                                           A0Result& result,
                                           double timeLimitMs) {
    std::cout << "[A0] Invocando reparación LP..." << std::endl;
    try {
        LpRepairHeuristic::Config repairConfig;
        repairConfig.timeLimitMs = std::min(timeLimitMs, 1000.0);

        auto repairResult = LpRepairHeuristic::repair(
            snapshot_, node.primal, node.fixedToOne, node.fixedToZero,
            *clpSolver_, repairConfig
        );

        if (repairResult.success) {
            std::cout << "[A0] Reparación exitosa: costo="
                      << repairResult.cost << std::endl;
            return updateIncumbent(repairResult.solution,
                                    repairResult.cost, result);
        } else {
            std::cout << "[A0] Reparación falló" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[A0] Reparación lanzó excepción: " << e.what() << std::endl;
    }
    return false;
}

// ============================================================================
// Helpers privados
// ============================================================================

double BranchAndBoundA0::getRemainingTime(double maxTimeSeconds) const {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - startTime_).count();
    double remaining = maxTimeSeconds - elapsed;
    return (remaining > 0.0) ? remaining : 0.0;
}

double BranchAndBoundA0::elapsedSeconds(
    const std::chrono::steady_clock::time_point& startTime) const {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - startTime).count();
}

double BranchAndBoundA0::computeGap(double ub, double lb) const {
    if (ub >= std::numeric_limits<double>::infinity()) return 0.0;
    return 100.0 * (ub - lb) / std::max(1.0, std::abs(ub));
}

} // namespace optimization