#ifndef OPTIMIZATION_SIMULATOR_H
#define OPTIMIZATION_SIMULATOR_H

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <cstdint>
#include <limits>

#include "domain/Grid.h"
#include "domain/Command.h"
#include "domain/Snapshot.h"
#include "optimization/SolverTypes.h"
#include "optimization/RunConfig.h"
#include "optimization/SolveResult.h"
#include "optimization/PlacementStrategy.h"

namespace optimization {

// ============================================================================
// Tipos de solvers
// ============================================================================

/**
 * @brief Tipos de solvers disponibles en la simulación RPE.
 */
enum class SolverType {
    A0,  ///< Branch-and-Bound propio (Clp)
    H1,  ///< Golosa estable
    H2,  ///< GRASP + búsqueda local
    R    ///< COIN-OR Cbc (referencia)
};

// ============================================================================
// Resultado de una época
// ============================================================================

/**
 * @brief Resultado de una época (para registrar en CSV).
 */
struct EpochResult {
    int epochIndex = 0;
    int timestamp = 0;
    SolverType solverUsed = SolverType::H1;
    std::string solverName;
    bool feasible = false;
    bool isValid = false;
    double objectiveValue = 0.0;
    double executionTimeSeconds = 0.0;
    Solution solution;

    EpochResult() = default;

    EpochResult(int epoch, const Solution& sol, double t, bool valid, double obj)
        : epochIndex(epoch),
          timestamp(0),
          solverUsed(SolverType::H1),
          feasible(valid),
          isValid(valid),
          objectiveValue(obj),
          executionTimeSeconds(t),
          solution(sol) {}
};

// ============================================================================
// Resultado de un batch atómico
// ============================================================================

/**
 * @brief Resultado de un batch atómico.
 */
struct BatchResult {
    bool success = false;
    size_t appliedCount = 0;
    size_t failedAt = 0;
    std::string errorMsg;
};

// ============================================================================
// Multi-semilla GRASP
// ============================================================================

/**
 * @brief Resultado de una corrida de GRASP con una semilla específica.
 */
struct GraspSeedResult {
    uint64_t seed = 0;
    int epoch = 0;
    double objective = std::numeric_limits<double>::infinity();
    double runtimeMs = 0.0;
    int feasibleConstructions = 0;
    bool bestFound = false;
    Solution solution;
};

/**
 * @brief Resultado consolidado de una evaluación multi-semilla de GRASP.
 */
struct GraspMultiSeedResult {
    std::vector<GraspSeedResult> perSeed;
    double bestCost = std::numeric_limits<double>::infinity();
    double worstCost = -std::numeric_limits<double>::infinity();
    double avgCost = 0.0;
    double stdDevCost = 0.0;
    double avgRuntimeMs = 0.0;
    int numSeeds = 0;
    int numFeasible = 0;
    Solution bestSolution;
    uint64_t bestSeed = 0;
};

// ============================================================================
// Simulator
// ============================================================================

/**
 * @brief Simulador de la evolución temporal del RPE.
 *
 * Responsabilidades:
 *  - Aplicar lotes de comandos atómicamente (con rollback).
 *  - Resolver cada época con los métodos configurados (Patrón Strategy).
 *  - Seleccionar la mejor solución factible (menor costo, desempate lex).
 *  - Actualizar la posición previa (p_i^{t+1}) y los costos (c_ij^{t+1}).
 *  - Ejecutar evaluación multi-semilla de GRASP (Sección 9).
 */
class Simulator {
public:
    /**
     * @brief Constructor.
     */
    Simulator(domain::Grid grid,
              domain::EpochCommands epochCommands,
              double graspEta,
              double graspLambda,
              SolverType solverType = SolverType::H1,
              bool runAllSolvers = false);

    /**
     * @brief Ejecuta la simulación completa época por época.
     */
    std::vector<EpochResult> run();

    // ========================================================================
    // Accesores públicos
    // ========================================================================

    const domain::Grid& getGrid() const noexcept { return grid_; }
    domain::Grid& getGridMutable() noexcept { return grid_; }

    uint64_t getBaseSeed() const noexcept { return baseSeed_; }
    int getMaxIterations() const noexcept { return maxIterations_; }
    double getGraspEta() const noexcept { return graspEta_; }
    double getGraspLambda() const noexcept { return graspLambda_; }

    // --- Configuración post-construcción ---
    void setBaseSeed(uint64_t seed) { baseSeed_ = seed; }
    void setMaxIterations(int iters) { maxIterations_ = iters; }
    void setRunAllSolvers(bool value) { runAllSolvers_ = value; }

    // ========================================================================
    // Aplicación atómica de comandos
    // ========================================================================

    /**
     * @brief Aplica un lote de comandos atómicamente con rollback.
     */
    BatchResult applyCommandsAtomically(const domain::CommandBatch& commands);

    // ========================================================================
    // Evaluación multi-semilla de GRASP (Sección 9)
    // ========================================================================

    /**
     * @brief Ejecuta GRASP (H2) con múltiples semillas sobre la época actual.
     *
     * Para cada semilla `s_i = baseSeed + i`, ejecuta H2 sobre un snapshot
     * fresco del grid actual (sin modificar el estado interno del Simulator).
     *
     * @param numSeeds Número de semillas a evaluar (mínimo 5 recomendado).
     * @param timeLimitMs Tiempo máximo por semilla en milisegundos.
     * @param epoch Época a evaluar (0 = estado inicial).
     * @return GraspMultiSeedResult Resumen estadístico consolidado.
     */
    GraspMultiSeedResult runGraspMultiSeed(int numSeeds = 5,
                                            int timeLimitMs = 1000,
                                            int epoch = 0) const;

private:
    // ========================================================================
    // Procesamiento de época
    // ========================================================================

    EpochResult processEpoch(int epoch, int timestamp);

    std::unique_ptr<PlacementStrategy> makeStrategy(SolverType type) const;

    RunConfig makeRunConfig(int epoch, int timeLimitMs) const;

    SolveResult selectBestResult(const domain::Snapshot& snapshot,
                                  int timeLimitMs,
                                  int epoch);

    static bool isBetterResult(const SolveResult& a,
                               const SolveResult& b,
                               double epsilonBound);

    Solution createEmptySolution(int numPieces) const;

    // ========================================================================
    // Miembros
    // ========================================================================
    domain::Grid grid_;
    domain::EpochCommands epochCommands_;
    double graspEta_;
    double graspLambda_;
    SolverType solverType_;
    bool runAllSolvers_;

    uint64_t baseSeed_;
    int maxIterations_;

    size_t totalEpochs_;
    size_t solvedEpochs_;
    Solution prevSolution_;
    std::vector<EpochResult> results_;
};

} // namespace optimization

#endif // OPTIMIZATION_SIMULATOR_H