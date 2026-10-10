#include "optimization/LocalSearch.h"
#include "domain/Validator.h"
#include <iostream>
#include <limits>
#include <algorithm>

namespace optimization {

Solution LocalSearch::optimize(
    const Solution& initialSol,
    const domain::Snapshot& snapshot,
    const Config& config
) {
    auto startTime = std::chrono::steady_clock::now();

    // Inicializar estado de la grilla para acceso O(1)
    GridState state;
    state.rows = snapshot.getGeometry().getRows();
    state.cols = snapshot.getGeometry().getCols();
    size_t totalCells = state.rows * state.cols;
    state.cellToPiece.assign(totalCells, -1);
    for (size_t pieceIndex = 0; pieceIndex < initialSol.size(); ++pieceIndex) {
        if (initialSol[pieceIndex] >= 0) {
            state.pieceToCell[static_cast<int>(pieceIndex)] = static_cast<size_t>(initialSol[pieceIndex]);
        }
    }

    for (std::unordered_map<int, size_t>::const_iterator it = state.pieceToCell.begin();
         it != state.pieceToCell.end(); ++it) {
        state.cellToPiece[it->second] = it->first;
    }

    double currentCost = calculateTotalCost(state, snapshot);
    std::vector<std::pair<int, size_t>> placedPieces;
    placedPieces.reserve(state.pieceToCell.size());

    // Bucle principal de Búsqueda Local
    while (true) {
        // Control de tiempo T_max
        auto now = std::chrono::steady_clock::now();
        double elapsedSeconds = std::chrono::duration<double>(now - startTime).count();
        if (elapsedSeconds >= config.maxTimeSeconds) {
            break;
        }

        NeighborMove bestMove;
        bestMove.delta = 0.0; // Buscamos un delta < 0 (mejora)

        // --- Generar Vecindario C = Relocate U Swap ---
        
        // 1. Explorar Movimientos RELOCATE
        for (std::unordered_map<int, size_t>::const_iterator pieceIt = state.pieceToCell.begin();
             pieceIt != state.pieceToCell.end(); ++pieceIt) {
            int pieceId = pieceIt->first;
            size_t currentCell = pieceIt->second;
            for (size_t targetCell = 0; targetCell < totalCells; ++targetCell) {
                if (targetCell == currentCell) continue;
                if (state.cellToPiece[targetCell] != -1) continue; // Solo celdas desocupadas

                double delta = evaluateRelocateDelta(pieceId, currentCell, targetCell, state, snapshot);

                if (delta < bestMove.delta) {
                    bestMove.type = NeighborMove::RELOCATE;
                    bestMove.piece1_id = pieceId;
                    bestMove.cell1_idx = currentCell;
                    bestMove.cell2_idx = targetCell;
                    bestMove.delta = delta;

                    if (config.useFirstImprovement) break;
                }
            }
            if (config.useFirstImprovement && bestMove.delta < 0) break;
        }

        // 2. Explorar Movimientos SWAP
        if (!config.useFirstImprovement || bestMove.delta >= 0) {
            placedPieces.clear();
            placedPieces.insert(placedPieces.end(), state.pieceToCell.begin(), state.pieceToCell.end());
            
            for (size_t i = 0; i < placedPieces.size(); ++i) {
                for (size_t j = i + 1; j < placedPieces.size(); ++j) {
                    int p1 = placedPieces[i].first;
                    size_t c1 = placedPieces[i].second;
                    int p2 = placedPieces[j].first;
                    size_t c2 = placedPieces[j].second;

                    double delta = evaluateSwapDelta(p1, c1, p2, c2, state, snapshot);

                    if (delta < bestMove.delta) {
                        bestMove.type = NeighborMove::SWAP;
                        bestMove.piece1_id = p1;
                        bestMove.cell1_idx = c1;
                        bestMove.piece2_id = p2;
                        bestMove.cell2_idx = c2;
                        bestMove.delta = delta;

                        if (config.useFirstImprovement) break;
                    }
                }
                if (config.useFirstImprovement && bestMove.delta < 0) break;
            }
        }

        // Criterio de parada: no hay ningún movimiento que mejore la solución (delta < 0)
        if (bestMove.type == NeighborMove::NONE || bestMove.delta >= -1e-9) {
            break;
        }

        // Aplicar el mejor movimiento encontrado
        applyMove(state, bestMove);
        currentCost += bestMove.delta;
    }

    Solution result(initialSol.size(), -1);
    for (std::unordered_map<int, size_t>::const_iterator it = state.pieceToCell.begin();
         it != state.pieceToCell.end(); ++it) {
        if (it->first >= 0 && static_cast<size_t>(it->first) < result.size()) {
            result[static_cast<size_t>(it->first)] = static_cast<int>(it->second);
        }
    }
    return result;
}

double LocalSearch::calculateCellConflicts(
    int pieceId, size_t cellIdx, const GridState& state, const domain::Snapshot& snapshot
) {
    if (pieceId == -1) return 0.0;

    // 1. Obtener puntero seguro a la pieza
    const domain::Piece* p = snapshot.findPiece(pieceId);
    if (!p) return 0.0;

    const auto& geometry = snapshot.getGeometry();
    int cellId = static_cast<int>(cellIdx);

    double conflicts = 0.0;

    // 2. Verificar Compatibilidad Estructural (Bordes del tablero vs Lados grises)
    const domain::OuterSides outerSides = geometry.getOuterSides(cellId);
    if (!domain::Validator::checkStructuralCompatibility(*p, outerSides)) {
        conflicts += 1.0;
    }

    // 3. Verificar Compatibilidad con Pistas de la Celda (si existen)
    const std::pair<domain::Side, int>* clue = snapshot.getCellClue(cellId);
    if (clue != nullptr) {
        if (p->getColor(clue->first) != clue->second) {
            conflicts += 1.0;
        }
    }

    // 4. Verificar Compatibilidad de Colores con Piezas Vecinas Adyacentes
    const domain::AdjacentCells adjacentCells = geometry.getAdjacentCells(cellId);
    for (int adjacentIndex = 0; adjacentIndex < adjacentCells.count; ++adjacentIndex) {
        int neighborCellId = adjacentCells.cells[adjacentIndex].first;
        int neighborPieceId = state.cellToPiece[neighborCellId];

        // Si la celda vecina tiene una pieza colocada
        if (neighborPieceId != -1) {
            domain::Side sideFromThis = adjacentCells.cells[adjacentIndex].second;
            domain::Side sideFromNeighbor = domain::getOppositeSide(sideFromThis);

            // Determinar el lado opuesto de la pieza vecina
            switch (sideFromThis) {
                case domain::Side::N: sideFromNeighbor = domain::Side::S; break;
                case domain::Side::E: sideFromNeighbor = domain::Side::O; break;
                case domain::Side::S: sideFromNeighbor = domain::Side::N; break;
                case domain::Side::O: sideFromNeighbor = domain::Side::E; break;
            }

            // Buscar la pieza vecina en el snapshot
            const domain::Piece* neighborPiece = snapshot.findPiece(neighborPieceId);
            if (neighborPiece != nullptr) {
                if (p->getColor(sideFromThis) != neighborPiece->getColor(sideFromNeighbor)) {
                    conflicts += 1.0; // Desajuste de color entre piezas
                }
            }
        }
    }

    return conflicts;
}

double LocalSearch::evaluateRelocateDelta(
    int pieceId, size_t fromCell, size_t toCell, const GridState& state, const domain::Snapshot& snapshot
) {
    // Para evaluar el delta, restamos los conflictos en la posición actual
    // y sumamos los conflictos hipotéticos en la nueva posición.
    double oldConflicts = calculateCellConflicts(pieceId, fromCell, state, snapshot);
    
    // Simulación rápida: cambiar temporalmente en la grilla
    GridState& nonConstState = const_cast<GridState&>(state);
    nonConstState.cellToPiece[fromCell] = -1;   // Saca la pieza de su posicion actual
    nonConstState.cellToPiece[toCell] = pieceId;    // Pone la pieza en la celda de destino vacia

    double newConflicts = calculateCellConflicts(pieceId, toCell, nonConstState, snapshot);

    // Revertir simulación
    nonConstState.cellToPiece[fromCell] = pieceId;
    nonConstState.cellToPiece[toCell] = -1;

    return newConflicts - oldConflicts;
}

double LocalSearch::evaluateSwapDelta(
    int p1, size_t c1, int p2, size_t c2, const GridState& state, const domain::Snapshot& snapshot
) {
    double oldConflicts = calculateCellConflicts(p1, c1, state, snapshot) +
                         calculateCellConflicts(p2, c2, state, snapshot);

    GridState& nonConstState = const_cast<GridState&>(state);
    nonConstState.cellToPiece[c1] = p2;
    nonConstState.cellToPiece[c2] = p1;

    double newConflicts = calculateCellConflicts(p1, c2, nonConstState, snapshot) +
                         calculateCellConflicts(p2, c1, nonConstState, snapshot);

    nonConstState.cellToPiece[c1] = p1;
    nonConstState.cellToPiece[c2] = p2;

    return newConflicts - oldConflicts;
}

double LocalSearch::calculateTotalCost(const GridState& state, const domain::Snapshot& snapshot) {
    double total = 0.0;
    for (size_t cell = 0; cell < state.cellToPiece.size(); ++cell) {
        int pId = state.cellToPiece[cell];
        if (pId != -1) {
            total += calculateCellConflicts(pId, cell, state, snapshot);
        }
    }
    return total / 2.0; // Cada conflicto entre dos celdas se suma 2 veces
}

void LocalSearch::applyMove(GridState& state, const NeighborMove& move) {
    if (move.type == NeighborMove::RELOCATE) {
        state.cellToPiece[move.cell1_idx] = -1;
        state.cellToPiece[move.cell2_idx] = move.piece1_id;
        state.pieceToCell[move.piece1_id] = move.cell2_idx;
    } else if (move.type == NeighborMove::SWAP) {
        state.cellToPiece[move.cell1_idx] = move.piece2_id;
        state.cellToPiece[move.cell2_idx] = move.piece1_id;
        state.pieceToCell[move.piece1_id] = move.cell2_idx;
        state.pieceToCell[move.piece2_id] = move.cell1_idx;
    }
}

} // namespace optimization