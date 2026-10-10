#include "optimization/HeuristicaH1.h"
#include "optimization/LocalSearch.h"
#include "domain/Validator.h"
#include <algorithm>
#include <tuple>
#include <chrono>
#include <limits>
#include <vector>

namespace optimization {

Solution HeuristicH1::solve(const domain::Snapshot& snapshot, 
                            const Solution& prevSolution,
                            double maxTimeSeconds) {
    auto startTime = std::chrono::steady_clock::now();
    
    const auto& pieces = snapshot.getPieces();
    const auto& cells = snapshot.getCells();
    const auto& geometry = snapshot.getGeometry();
    
    const size_t numPieces = pieces.size();
    const size_t numCells = cells.size();
    
    // Vector de salida X: indexado por pieceId -> cellId (-1 si no está ubicada)
    Solution X(numPieces, -1);
    
    // Arreglo inverso para lookup O(1) de ocupación: cellId -> pieceId (-1 si está libre)
    std::vector<int> cellToPiece(numCells, -1);
    
    // Conjunto U de piezas no ubicadas
    std::vector<int> U;
    U.reserve(numPieces);
    for (size_t i = 0; i < numPieces; ++i) {
        U.push_back(static_cast<int>(i));
    }

    // BUCLE PRINCIPAL: MIENTRAS U != ∅
    while (!U.empty()) {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsed >= maxTimeSeconds) {
            return {};
        }

        int bestPiece = -1;
        std::tuple<size_t, double, int> bestPieceTuple{
            std::numeric_limits<size_t>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<int>::max()
        };
        
        std::vector<int> bestPieceF;

        // PARA CADA i in U: Calcular F(i)
        for (size_t uIdx = 0; uIdx < U.size(); ++uIdx) {
            int i = U[uIdx];
            const auto& pieceI = pieces[i];
            
            std::vector<int> Fi;
            Fi.reserve(numCells);

            for (size_t j = 0; j < numCells; ++j) {
                // 1. Celda libre
                if (cellToPiece[j] != -1) continue;

                // 2. Compatibilidad estructural Gi == Oj
                const domain::OuterSides outerSides = geometry.getOuterSides(j);
                if (!domain::Validator::checkStructuralCompatibility(pieceI, outerSides)) {
                    continue;
                }

                // 3. Compatibilidad con pista
                const auto* clue = snapshot.getCellClue(j);
                if (clue != nullptr && pieceI.getColor(clue->first) != clue->second) {
                    continue;
                }

                // 4. Compatibilidad con vecinos
                bool fitsNeighbors = true;
                const auto adjacentCells = geometry.getAdjacentCells(j);

                for (int adjIdx = 0; adjIdx < adjacentCells.count; ++adjIdx) {
                    const auto& adj = adjacentCells.cells[adjIdx];
                    int neighborCellId = adj.first;
                    domain::Side sideFromThis = adj.second;
                    int neighborPieceIndex = cellToPiece[neighborCellId];

                    if (neighborPieceIndex != -1) {
                        domain::Side sideFromNeighbor;
                        switch (sideFromThis) {
                            case domain::Side::N: sideFromNeighbor = domain::Side::S; break;
                            case domain::Side::E: sideFromNeighbor = domain::Side::O; break;
                            case domain::Side::S: sideFromNeighbor = domain::Side::N; break;
                            case domain::Side::O: sideFromNeighbor = domain::Side::E; break;
                            default: continue;
                        }

                        if (pieceI.getColor(sideFromThis) != 
                            pieces[neighborPieceIndex].getColor(sideFromNeighbor)) {
                            fitsNeighbors = false;
                            break;
                        }
                    }
                }

                if (fitsNeighbors) {
                    Fi.push_back(static_cast<int>(j));
                }
            }

            // Tupla lexicográfica: (|F(i)|, -s_i, i)
            double negCost = -pieceI.getCost();
            std::tuple<size_t, double, int> currentTuple{Fi.size(), negCost, i};

            if (currentTuple < bestPieceTuple) {
                bestPieceTuple = currentTuple;
                bestPiece = i;
                bestPieceF = std::move(Fi);
            }
        }

        if (bestPiece == -1 || bestPieceF.empty()) {
            return {};
        }

        // Encontrar j* = argmin_{j in F(i*)} (c^t_{i*j}, j)
        int bestCell = -1;
        std::tuple<double, int> bestCellTuple{
            std::numeric_limits<double>::max(),
            std::numeric_limits<int>::max()
        };

        for (int j : bestPieceF) {
            // FIX #4: solo cobrar si había celda previa válida
            double transitionCost = 0.0;
            if (static_cast<size_t>(bestPiece) < prevSolution.size() && 
                prevSolution[bestPiece] >= 0 &&               // ← CELDA PREVIA VÁLIDA
                prevSolution[bestPiece] != j) {
                transitionCost = pieces[bestPiece].getCost();
            }

            std::tuple<double, int> currentCellTuple{transitionCost, j};
            if (currentCellTuple < bestCellTuple) {
                bestCellTuple = currentCellTuple;
                bestCell = j;
            }
        }

        // Aplicar asignaciones
        X[bestPiece] = bestCell;
        cellToPiece[bestCell] = bestPiece;

        // ELIMINAR i* DE U (Swap & Pop)
        auto it = std::find(U.begin(), U.end(), bestPiece);
        if (it != U.end()) {
            *it = U.back();
            U.pop_back();
        }
    }

    // Búsqueda Local
    auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - startTime).count();
    double remainingTime = maxTimeSeconds - elapsed;

    if (remainingTime > 0.0) {
        LocalSearch::Config lsConfig;
        lsConfig.maxTimeSeconds = remainingTime;
        lsConfig.useFirstImprovement = false;

        X = LocalSearch::optimize(X, snapshot, lsConfig);
    }

    // Validación final
    if (domain::Validator::validate(X, snapshot).first) {
        return X;
    }

    return {};
}

} // namespace optimization