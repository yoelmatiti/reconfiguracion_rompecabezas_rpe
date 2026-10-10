#ifndef OPTIMIZATION_PLACEMENTSTRATEGY_H
#define OPTIMIZATION_PLACEMENTSTRATEGY_H

#include "domain/Snapshot.h"
#include "optimization/RunConfig.h"
#include "optimization/SolveResult.h"

namespace optimization {

/**
 * @brief Interfaz abstracta del Patrón Strategy para solvers RPE.
 *
 * Todos los métodos (A0, H1, H2, R) implementan esta interfaz, permitiendo
 * intercambiarlos polimórficamente en tiempo de ejecución sin acoplar el
 * Simulator a clases concretas.
 *
 * Contrato:
 *   - `solve` es determinista dado el mismo snapshot y config.
 *   - `solve` NO modifica el snapshot (es const).
 *   - `solve` retorna un `SolveResult` con status, solution y métricas.
 *   - `solve` NO debe lanzar excepciones; capturarlas internamente y
 *     retornar `SolveStatus::SOLVER_ERROR`.
 *
 * Uso típico:
 * @code
 *     std::unique_ptr<PlacementStrategy> strat(
 *         new A0Strategy(snapshot));
 *     RunConfig cfg;
 *     cfg.maxTimeSeconds = 10.0;
 *     SolveResult res = strat->solve(snapshot, cfg);
 *     if (res.hasSolution()) {
 *         // ...
 *     }
 * @endcode
 */
class PlacementStrategy {
public:
    virtual ~PlacementStrategy() = default;

    /**
     * @brief Resuelve una época con la estrategia.
     *
     * @param snapshot Snapshot inmutable del estado actual.
     * @param config Configuración de ejecución.
     * @return SolveResult Resultado normalizado.
     */
    virtual SolveResult solve(const domain::Snapshot& snapshot,
                              const RunConfig& config) = 0;

    /**
     * @brief Nombre legible de la estrategia (para logs y CSV).
     */
    virtual const char* name() const = 0;
};

} // namespace optimization

#endif // OPTIMIZATION_PLACEMENTSTRATEGY_H