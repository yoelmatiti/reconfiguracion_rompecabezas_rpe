#ifndef OPTIMIZATION_RUNCONFIG_H
#define OPTIMIZATION_RUNCONFIG_H

#include <cstdint>
#include <limits>

namespace optimization {

/**
 * @brief Configuración unificada para ejecutar una estrategia.
 *
 * Agrupa todos los parámetros que una estrategia necesita, evitando
 * firmas con muchos argumentos y facilitando la extensibilidad.
 */
struct RunConfig {
    /// Tiempo máximo de ejecución en segundos.
    double maxTimeSeconds = 30.0;

    /// Semilla base para GRASP (H2). El epoch se suma internamente.
    uint64_t baseSeed = 12345ULL;

    /// Época actual (0 para estado inicial). Usado en seed efectiva.
    int epoch = 0;

    /// Tolerancia de integralidad (fracciones en [εint, 1-εint] son no enteras).
    double epsilonInt = 1e-4;

    /// Tolerancia para comparación de cotas.
    double epsilonBound = 1e-8;

    /// Número máximo de iteraciones para H2 (GRASP).
    int maxIterations = 100;

    /// Parámetro η de RCL para H2 (∈ [0,1]).
    double graspEta = 0.5;

    /// Parámetro λ de riesgo para H2 (≥ 0).
    double graspLambda = 1.0;

    /// Nivel de logs del solver (0 = silencioso).
    int logLevel = 0;

    /// Activar heurística de reparación interna en A0.
    bool useRepairHeuristic = true;

    /// Activar warm start con H1 en A0.
    bool useWarmStartH1 = true;

    RunConfig() = default;
};

} // namespace optimization

#endif // OPTIMIZATION_RUNCONFIG_H