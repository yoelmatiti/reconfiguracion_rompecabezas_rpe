#include "optimization/HeuristicaH2.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <iostream>
#include <chrono>
#include <random>

namespace optimization {

namespace {
    // Constante de tolerancia para comparación de costos
    const double EPS = 1e-9;
}

// ============================================================================
// Punto de entrada principal
// ============================================================================

Solution HeuristicH2::solve(
    const domain::Snapshot& snapshot,
    const Params& params
) {
    // -------------------------------------------------------------------------
    // 1. Validación estricta de dominios de parámetros
    // -------------------------------------------------------------------------
    if (!std::isfinite(params.eta) || params.eta < 0.0 || params.eta > 1.0) {
        std::cerr << "[H2] Error: eta fuera de dominio [0,1]." << std::endl;
        return {};  // INVALID_CONFIG
    }
    if (!std::isfinite(params.lambda) || params.lambda < 0.0) {
        std::cerr << "[H2] Error: lambda fuera de dominio [0, inf)." << std::endl;
        return {};  // INVALID_CONFIG
    }
    if (params.maxTimeSeconds <= 0.0) {
        std::cerr << "[H2] Error: maxTimeSeconds debe ser positivo." << std::endl;
        return {};
    }
    if (params.maxIterations <= 0) {
        std::cerr << "[H2] Error: maxIterations debe ser positivo." << std::endl;
        return {};
    }

    // -------------------------------------------------------------------------
    // 2. Inicialización de PRNG y temporizador
    // -------------------------------------------------------------------------
    const auto startTime = std::chrono::steady_clock::now();

    // Semilla efectiva: seed + t (aritmética uint64_t, definida módulo 2^64)
    const uint64_t effectiveSeed = params.baseSeed + static_cast<uint64_t>(params.epoch);
    std::mt19937_64 rng(effectiveSeed);

    // -------------------------------------------------------------------------
    // 3. Precomputar costos máximos y geometría (una sola vez)
    // -------------------------------------------------------------------------
    const auto& pieces = snapshot.getPieces();
    const auto& geometry = snapshot.getGeometry();
    const size_t numPieces = pieces.size();
    const size_t totalCells = geometry.getRows() * geometry.getCols();

    // Costo máximo entre las piezas: max_{k in I} s_k
    double maxMovementCost = 1.0;
    for (size_t i = 0; i < numPieces; ++i) {
        const double s_i = pieces[i].getCost();
        if (s_i > maxMovementCost) {
            maxMovementCost = s_i;
        }
    }

    // -------------------------------------------------------------------------
    // 4. Precomputar celdas compatibles por pieza (compatibilidad estructural)
    // -------------------------------------------------------------------------
    std::vector<std::vector<size_t>> compatibleCells(numPieces);
    for (size_t i = 0; i < numPieces; ++i) {
        compatibleCells[i].reserve(totalCells / 2 + 1);
        for (size_t j = 0; j < totalCells; ++j) {
            const domain::OuterSides outerSides = geometry.getOuterSides(static_cast<int>(j));
            if (domain::Validator::checkStructuralCompatibility(pieces[i], outerSides)) {
                compatibleCells[i].push_back(j);
            }
        }
    }

    // -------------------------------------------------------------------------
    // 5. Estructuras reutilizables (evitar reallocaciones en el bucle)
    // -------------------------------------------------------------------------
    std::vector<int> U;                          // Piezas no asignadas (índices)
    U.reserve(numPieces);

    std::unordered_map<int, std::vector<size_t>> allF;  // F(i) por pieza (índice)
    allF.reserve(numPieces);
    for (size_t i = 0; i < numPieces; ++i) {
        allF[static_cast<int>(i)].reserve(compatibleCells[i].size());
    }

    std::vector<int> cellOccupiedBy(totalCells, -1);  // cellIdx -> pieceIdx
    std::vector<size_t> RCL;
    RCL.reserve(totalCells);

    std::vector<std::pair<size_t, double>> hValues;
    hValues.reserve(totalCells);

    // Solución actual como vector (pieza -> celda)
    std::vector<int> X(numPieces, -1);

    // Mejor solución global
    Solution Xbest(numPieces, -1);
    double zbest = std::numeric_limits<double>::infinity();
    uint64_t iterations = 0;
    uint64_t feasibleConstructions = 0;

    // -------------------------------------------------------------------------
    // 6. Bucle principal GRASP
    // -------------------------------------------------------------------------
    while (iterations < static_cast<uint64_t>(params.maxIterations)) {
        // --- 6.1 Verificar tiempo restante ---
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - startTime).count();
        if (elapsed >= params.maxTimeSeconds) {
            break;
        }

        // --- 6.2 Resetear estructuras para nueva construcción ---
        std::fill(X.begin(), X.end(), -1);
        std::fill(cellOccupiedBy.begin(), cellOccupiedBy.end(), -1);
        U.clear();
        for (size_t i = 0; i < numPieces; ++i) {
            U.push_back(static_cast<int>(i));
        }

        bool constructionValid = true;

        // --- 6.3 Bucle de construcción golosa aleatorizada ---
        while (!U.empty()) {
            const auto innerNow = std::chrono::steady_clock::now();
            const double innerElapsed = std::chrono::duration<double>(innerNow - startTime).count();
            if (innerElapsed >= params.maxTimeSeconds) {
                constructionValid = false;
                break;
            }

            // --- 6.3.1 Calcular F(i) para toda i ∈ U ---
            for (auto& entry : allF) {
                entry.second.clear();
            }

            for (int pieceIdx : U) {
                const auto& compat = compatibleCells[pieceIdx];
                auto& F_i = allF[pieceIdx];
                for (size_t cellIdx : compat) {
                    if (cellOccupiedBy[cellIdx] == -1) {
                        F_i.push_back(cellIdx);
                    }
                }
            }

            // --- 6.3.2 Seleccionar pieza i* = argmin_{i ∈ U} (|F(i)|, -s_i, i) ---
            int bestPieceIdx = -1;
            size_t bestSizeF = std::numeric_limits<size_t>::max();
            double bestNegCost = -std::numeric_limits<double>::infinity();
            int bestIdxTie = std::numeric_limits<int>::max();

            for (int pieceIdx : U) {
                const size_t sizeF = allF[pieceIdx].size();
                const double negCost = -pieces[pieceIdx].getCost();

                bool isBetter = false;
                if (sizeF < bestSizeF) {
                    isBetter = true;
                } else if (sizeF == bestSizeF) {
                    if (negCost > bestNegCost) {
                        isBetter = true;
                    } else if (negCost == bestNegCost && pieceIdx < bestIdxTie) {
                        isBetter = true;
                    }
                }

                if (isBetter) {
                    bestPieceIdx = pieceIdx;
                    bestSizeF = sizeF;
                    bestNegCost = negCost;
                    bestIdxTie = pieceIdx;
                }
            }

            // --- 6.3.3 Si F(i*) vacío, callejón sin salida ---
            if (bestPieceIdx < 0 || allF[bestPieceIdx].empty()) {
                constructionValid = false;
                break;
            }

            const auto& F_best = allF[bestPieceIdx];

            // --- 6.3.4 Calcular h(i*, j), hmin, hmax ---
            double hmin = std::numeric_limits<double>::infinity();
            double hmax = -std::numeric_limits<double>::infinity();

            hValues.clear();
            for (size_t cellJ : F_best) {
                const double hVal = calculateH(
                    bestPieceIdx, cellJ, U, allF, snapshot, maxMovementCost, params.lambda
                );
                hValues.push_back(std::make_pair(cellJ, hVal));
                if (hVal < hmin) hmin = hVal;
                if (hVal > hmax) hmax = hVal;
            }

            // --- 6.3.5 Construir RCL(η) ---
            const double cutoff = hmin + params.eta * (hmax - hmin);
            RCL.clear();
            for (size_t k = 0; k < hValues.size(); ++k) {
                if (hValues[k].second <= cutoff + EPS) {
                    RCL.push_back(hValues[k].first);
                }
            }

            if (RCL.empty()) {
                constructionValid = false;
                break;
            }

            // --- 6.3.6 Selección aleatoria uniforme de RCL ---
            std::uniform_int_distribution<size_t> dist(0, RCL.size() - 1);
            const size_t selectedCell = RCL[dist(rng)];

            // --- 6.3.7 Asignar y eliminar i* de U ---
            X[bestPieceIdx] = static_cast<int>(selectedCell);
            cellOccupiedBy[selectedCell] = bestPieceIdx;

            // Eliminar bestPieceIdx de U (swap-and-pop)
            for (size_t u = 0; u < U.size(); ++u) {
                if (U[u] == bestPieceIdx) {
                    U[u] = U.back();
                    U.pop_back();
                    break;
                }
            }
        }

        if (!U.empty()) constructionValid = false;

        // --- 6.4 Búsqueda local si la construcción fue válida ---
        if (constructionValid) {
            feasibleConstructions++;

            // Verificar tiempo restante
            const auto nowLS = std::chrono::steady_clock::now();
            const double elapsedLS = std::chrono::duration<double>(nowLS - startTime).count();
            const double remTime = params.maxTimeSeconds - elapsedLS;

            if (remTime > 0.0) {
                LocalSearch::Config lsConfig;
                lsConfig.maxTimeSeconds = remTime;
                lsConfig.useFirstImprovement = false;

                X = LocalSearch::optimize(X, snapshot, lsConfig);
            }

            // --- 6.5 Validar solución ---
            std::unordered_map<int, int> validatedSolution;
            validatedSolution.reserve(numPieces);
            for (size_t i = 0; i < numPieces; ++i) {
                if (X[i] >= 0) {
                    validatedSolution[pieces[i].getId()] = X[i];
                }
            }

            const std::pair<bool, double> validation =
                domain::Validator::validate(validatedSolution, snapshot);

            if (validation.first && validation.second < zbest - EPS) {
                Xbest = X;
                zbest = validation.second;

                // --- 6.5.1 PARADA TEMPRANA: costo 0 ---
                if (zbest <= EPS) {
                    break;
                }
            }
        }

        iterations++;

        if (zbest <= EPS) {
            break;
        }
    }

    // -------------------------------------------------------------------------
    // 7. Registro final de métricas
    // -------------------------------------------------------------------------
    std::cout << "[GRASP H2] Epoca: " << params.epoch
              << " | Seed base: " << params.baseSeed
              << " | Seed efectiva: " << effectiveSeed
              << " | Iteraciones: " << iterations
              << " | Construcciones factibles: " << feasibleConstructions
              << " | zbest: " << zbest << std::endl;

    return Xbest;
}

// ============================================================================
// Cálculo de h(i, j) = c_ij / max_k(s_k) + λ · riesgo(i, j)
// ============================================================================

double HeuristicH2::calculateH(
    int pieceIdx,
    size_t cellIdx,
    const std::vector<int>& U,
    const std::unordered_map<int, std::vector<size_t>>& allF,
    const domain::Snapshot& snapshot,
    double maxMovementCost,
    double lambda
) {
    const int pieceId = snapshot.getPieces()[pieceIdx].getId();

    // Componente de costo normalizado
    const double c_ij = snapshot.getMoveCost(pieceId, static_cast<int>(cellIdx));
    const double normalizedCost = c_ij / maxMovementCost;

    // Componente de riesgo: número de piezas i' ∈ U \ {i} cuya única opción es j
    if (lambda <= 0.0) {
        return normalizedCost;
    }

    double riesgo = 0.0;
    for (int otherPieceIdx : U) {
        if (otherPieceIdx == pieceIdx) continue;

        const auto it = allF.find(otherPieceIdx);
        if (it != allF.end()) {
            const auto& fOther = it->second;
            if (fOther.size() == 1 && fOther[0] == cellIdx) {
                riesgo += 1.0;
            }
        }
    }

    return normalizedCost + lambda * riesgo;
}

} // namespace optimization