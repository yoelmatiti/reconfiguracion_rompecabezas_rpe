#ifndef OPTIMIZATION_HEURISTICH1_H
#define OPTIMIZATION_HEURISTICH1_H

#include <vector>
#include "domain/Snapshot.h"
#include "optimization/SolverTypes.h"

namespace optimization {

/**
 * @brief Heurística Golosa Estable (H1) para el problema RPE.
 * 
 * Implementa un algoritmo determinista que selecciona en cada paso la pieza
 * no ubicada más crítica |F(i)| y la asigna a la celda que minimiza el costo
 * de transición c^t_{ij}, respetando el tiempo máximo asignado.
 * 
 * Complejidad: O(P^2 * V) donde P = número de piezas, V = número de celdas vecinas (≤ 4).
 */
class HeuristicH1 {
public:
    /**
     * @brief Resuelve una época usando la heurística golosa estable H1.
     * 
     * @param snapshot Snapshot inmutable del estado actual.
     * @param prevSolution Solución de la época anterior (x^{t-1}).
     * @param maxTimeSeconds Tiempo máximo de ejecución disponible T_max en segundos (por defecto 60.0).
     * @return Solution Solución X encontrada, o vector vacío si resulta NO_SOLUTION o expira el tiempo.
     */
    static Solution solve(const domain::Snapshot& snapshot, 
                          const Solution& prevSolution,
                          double maxTimeSeconds = 60.0);
};

} // namespace optimization

#endif // OPTIMIZATION_HEURISTICH1_H