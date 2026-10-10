#ifndef OPTIMIZATION_SOLVERESULT_H
#define OPTIMIZATION_SOLVERESULT_H

#include <string>
#include <unordered_map>
#include <vector>
#include <limits>
#include <cstdint>

#include "optimization/SolverTypes.h"

namespace optimization {

/**
 * @brief Resultado unificado de cualquier estrategia de colocación.
 *
 * Contiene la solución (si es factible), su costo, y las cotas
 * inferior y superior (para métodos exactos como A0 y R).
 */
struct SolveResult {
    SolveStatus status = SolveStatus::SOLVER_ERROR;

    /// Solución: índice = pieceIdx, valor = cellId (-1 si no asignada).
    Solution solution;

    /// Costo objetivo Z^t(x). Infinito si no hay solución.
    double objective = std::numeric_limits<double>::infinity();

    /// Cota inferior (solo para A0 y R). -inf si no aplica.
    double lowerBound = -std::numeric_limits<double>::infinity();

    /// Cota superior (= objective si hay incumbente).
    double upperBound = std::numeric_limits<double>::infinity();

    /// Gap % (solo para A0 y R).
    double gapPercent = 0.0;

    /// Tiempo de ejecución en segundos.
    double executionTimeSeconds = 0.0;

    /// Nombre del método (para logging).
    std::string methodName;

    /// Mensaje descriptivo (opcional).
    std::string message;

    // --- Estadísticas opcionales ---
    int nodesExplored = 0;
    int lpCalls = 0;
    int simplexIterations = 0;
    int prunedByBound = 0;
    int prunedByInfeasibility = 0;

    /**
     * @brief Verifica si el resultado tiene solución factible válida.
     */
    bool hasSolution() const {
        return status == SolveStatus::OPTIMAL ||
               status == SolveStatus::FEASIBLE;
    }

    /**
     * @brief Verifica si hay cota inferior disponible (métodos exactos).
     */
    bool hasLowerBound() const {
        return lowerBound > -std::numeric_limits<double>::infinity();
    }
};

} // namespace optimization

#endif // OPTIMIZATION_SOLVERESULT_H