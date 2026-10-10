#ifndef OPTIMIZATION_CSVEXPORTER_H
#define OPTIMIZATION_CSVEXPORTER_H

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace optimization {

// ============================================================================
// CSVRow — fila del CSV principal (Sección 7)
// ============================================================================

/**
 * @brief Fila del CSV principal de resultados por época y método.
 *
 * Formato estricto de la Sección 7 del enunciado:
 * epoch,timestamp,method,status,objective,moved_pieces,runtime_ms,
 * LB,UB,gap_percent,nodes,lp_calls,placement
 */
struct CSVRow {
    int epoch = 0;                  ///< Número de época (0, 1, 2, ...)
    int timestamp = 0;              ///< Marca temporal θ asociada
    std::string method;             ///< A0 | H1 | H2 | R | SIMULATOR
    std::string status;             ///< OPTIMAL | FEASIBLE | INFEASIBLE | ...
    double objective = 0.0;         ///< Valor Z(x). Inf → "inf"
    int moved_pieces = 0;           ///< Piezas que cambiaron respecto a t-1
    double runtime_ms = 0.0;        ///< Tiempo de ejecución (ms)
    double LB = 0.0;                ///< Cota inferior (0.0 para heurísticas)
    double UB = 0.0;                ///< Cota superior
    double gap_percent = 0.0;       ///< % gap (0.0 para heurísticas)
    int nodes = 0;                  ///< Nodos explorados por B&B
    int lp_calls = 0;               ///< Llamadas al solver LP
    std::string placement;          ///< "0->5;1->2;2->7" ordenado
};

// ============================================================================
// GraspSeedRow — fila del CSV dedicado a evaluación multi-semilla
// ============================================================================

/**
 * @brief Fila del CSV dedicado a la evaluación multi-semilla de GRASP.
 *
 * Formato:
 * seed,epoch,objective,runtime_ms,feasible_constructions,best_found
 *
 * Usado por la Sección 9 del enunciado para analizar la variabilidad
 * de GRASP sobre al menos 5 semillas distintas.
 */
struct GraspSeedRow {
    uint64_t seed = 0;              ///< Semilla base usada en esta corrida
    int epoch = 0;                  ///< Época evaluada
    double objective = 0.0;         ///< Costo Z(x) hallado (inf si no factible)
    double runtimeMs = 0.0;         ///< Tiempo de ejecución (ms)
    int feasibleConstructions = 0;  ///< 1 si encontró solución, 0 si no
    bool bestFound = false;         ///< true si esta semilla dio el mejor Z
};

// ============================================================================
// CSVExporter
// ============================================================================

/**
 * @brief Exportador de resultados a CSV.
 *
 * Responsabilidades:
 *  - Formatear placement en orden ascendente por pieceId.
 *  - Escapar valores de texto que contengan comas/comillas.
 *  - Formatear doubles con precisión fija (6 decimales).
 *  - Manejar "inf" para objetivos no acotados.
 */
class CSVExporter {
public:
    /**
     * @brief Escribe el CSV principal de resultados.
     *
     * Cabecera exacta:
     * epoch,timestamp,method,status,objective,moved_pieces,runtime_ms,
     * LB,UB,gap_percent,nodes,lp_calls,placement
     */
    static bool writeResults(const std::string& path,
                             const std::vector<CSVRow>& rows);

    /**
     * @brief Escribe el CSV dedicado a evaluación multi-semilla de GRASP.
     *
     * Cabecera exacta:
     * seed,epoch,objective,runtime_ms,feasible_constructions,best_found
     */
    static bool writeGraspMultiSeed(
        const std::string& path,
        const std::vector<GraspSeedRow>& rows);

    /**
     * @brief Convierte un mapa (pieceId -> cellId) a una cadena ordenada.
     *
     * Formato: "0->5;1->2;2->7" con pieceIds en orden ascendente.
     */
    static std::string placementToString(
        const std::unordered_map<int, int>& placement);
};

} // namespace optimization

#endif // OPTIMIZATION_CSVEXPORTER_H