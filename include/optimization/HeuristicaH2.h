#ifndef OPTIMIZATION_HEURISTICAH2_H
#define OPTIMIZATION_HEURISTICAH2_H

#include <unordered_map>
#include <vector>
#include <cstdint>
#include <limits>
#include "domain/Snapshot.h"
#include "domain/Validator.h"
#include "optimization/SolverTypes.h"
#include "optimization/LocalSearch.h"

namespace optimization {

/**
 * @brief Heurística GRASP (Greedy Randomized Adaptive Search Procedure) para RPE.
 * 
 * Genera soluciones constructivas aleatorizadas y las mejora con búsqueda local.
 * La función h(i, j) = c_ij / max_k(s_k) + λ · riesgo(i, j) guía la construcción.
 */
class HeuristicH2 {
public:
    /**
     * @brief Parámetros de control del GRASP.
     */
    struct Params {
        double eta = 0.5;              ///< η ∈ [0, 1] para RCL
        double lambda = 1.0;           ///< λ ∈ [0, ∞) para riesgo
        uint64_t baseSeed = 12345;     ///< Semilla base del archivo .rpe
        uint32_t epoch = 0;            ///< Época t (seed efectiva = baseSeed + t)
        double maxTimeSeconds = 60.0;  ///< Límite de tiempo en segundos
        int maxIterations = 100;       ///< Límite de iteraciones (NUEVO)
    };

    /**
     * @brief Ejecuta H2 (GRASP + Búsqueda Local).
     * 
     * @param snapshot Snapshot inmutable de la época t.
     * @param params Parámetros de control.
     * @return Asignación (pieceId → cellIdx) o mapa vacío si NO_SOLUTION.
     */
    static Solution solve(
        const domain::Snapshot& snapshot,
        const Params& params
    );

private:
    /**
     * @brief Calcula h(i, j) = c_ij / max_k(s_k) + λ · riesgo(i, j).
     * 
     * @param pieceIdx Índice de la pieza en el vector de piezas.
     * @param cellIdx Índice de la celda candidata.
     * @param U Índices de piezas no asignadas.
     * @param allF F(i) indexado por pieceIdx.
     * @param snapshot Snapshot de referencia.
     * @param maxMovementCost max_k(s_k).
     * @param lambda Peso del riesgo.
     * @return double Valor de h(i, j).
     */
    static double calculateH(
        int pieceIdx,
        size_t cellIdx,
        const std::vector<int>& U,
        const std::unordered_map<int, std::vector<size_t>>& allF,
        const domain::Snapshot& snapshot,
        double maxMovementCost,
        double lambda
    );
};

} // namespace optimization

#endif // OPTIMIZATION_HEURISTICAH2_H