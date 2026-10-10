#include "optimization/CSVExporter.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

namespace optimization {

namespace {

/**
 * @brief Escapa un valor de texto para CSV.
 *
 * Si el valor contiene comas o comillas, se envuelve entre comillas dobles
 * y se duplican las comillas internas (RFC 4180).
 */
std::string sanitize(const std::string& value) {
    if (value.find(',') == std::string::npos &&
        value.find('"') == std::string::npos &&
        value.find('\n') == std::string::npos) {
        return value;
    }

    std::string escaped;
    escaped.reserve(value.size() + 2);
    escaped += '"';
    for (char ch : value) {
        if (ch == '"') escaped += '"';  // Duplicar comillas
        escaped += ch;
    }
    escaped += '"';
    return escaped;
}

/**
 * @brief Formatea un double, usando "inf" para infinito.
 */
std::string formatDouble(double v) {
    if (std::isinf(v)) {
        return (v > 0) ? "inf" : "-inf";
    }
    if (std::isnan(v)) {
        return "nan";
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << v;
    return oss.str();
}

} // namespace

// ============================================================================
// placementToString — ordenado por pieceId ascendente
// ============================================================================

/**
 * @brief Convierte un mapa (pieceId -> cellId) a una cadena ordenada.
 *
 * Formato: "0->5;1->2;2->7" con pieceIds en orden ascendente.
 *
 * Se ordena explícitamente porque `unordered_map` no garantiza orden de
 * iteración, y el formato CSV de la Sección 7 exige orden ascendente.
 */
std::string CSVExporter::placementToString(
    const std::unordered_map<int, int>& placement) {

    if (placement.empty()) return "";

    // Extraer claves y ordenarlas.
    std::vector<int> keys;
    keys.reserve(placement.size());
    for (const auto& pair : placement) {
        keys.push_back(pair.first);
    }
    std::sort(keys.begin(), keys.end());

    // Construir la cadena en orden.
    std::ostringstream oss;
    bool first = true;
    for (int pieceId : keys) {
        if (!first) oss << ";";
        oss << pieceId << "->" << placement.at(pieceId);
        first = false;
    }
    return oss.str();
}

// ============================================================================
// writeResults — formato estricto de la Sección 7
// ============================================================================

bool CSVExporter::writeResults(const std::string& path,
                               const std::vector<CSVRow>& rows) {
    std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }

    // Cabecera estricta de la Sección 7.
    out << "epoch,timestamp,method,status,objective,moved_pieces,"
           "runtime_ms,LB,UB,gap_percent,nodes,lp_calls,placement\n";

    for (const auto& row : rows) {
        out << row.epoch << ','
            << row.timestamp << ','
            << sanitize(row.method) << ','
            << sanitize(row.status) << ','
            << formatDouble(row.objective) << ','
            << row.moved_pieces << ','
            << formatDouble(row.runtime_ms) << ','
            << formatDouble(row.LB) << ','
            << formatDouble(row.UB) << ','
            << formatDouble(row.gap_percent) << ','
            << row.nodes << ','
            << row.lp_calls << ','
            << sanitize(row.placement) << '\n';
    }

    return true;
}

// ============================================================================
// writeGraspMultiSeed — CSV dedicado para experimentos multi-semilla
// ============================================================================

/**
 * @brief Escribe el CSV de evaluación multi-semilla de GRASP.
 *
 * Formato: seed,epoch,objective,runtime_ms,feasible_constructions,best_found
 */
bool CSVExporter::writeGraspMultiSeed(
    const std::string& path,
    const std::vector<GraspSeedRow>& rows) {

    std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }

    out << "seed,epoch,objective,runtime_ms,feasible_constructions,best_found\n";

    for (const auto& row : rows) {
        out << row.seed << ','
            << row.epoch << ','
            << formatDouble(row.objective) << ','
            << formatDouble(row.runtimeMs) << ','
            << row.feasibleConstructions << ','
            << (row.bestFound ? "true" : "false") << '\n';
    }

    return true;
}

} // namespace optimization