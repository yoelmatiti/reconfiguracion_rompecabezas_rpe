// optimization/SolverTypes.h
#ifndef OPTIMIZATION_SOLVERTYPES_H
#define OPTIMIZATION_SOLVERTYPES_H

#include <string>
#include <unordered_map>
#include <vector>
#include <limits>

namespace optimization {

using Solution = std::vector<int>;

// ============================================================================
// Estados de solver
// ============================================================================

enum class SolverStatus {
    OPTIMAL,
    FEASIBLE,
    INFEASIBLE,
    LIMIT,
    INVALID,
    NO_INCUMBENT,
    ABANDONED,
    SOLVER_ERROR
};

enum class SolveStatus {
    OPTIMAL,
    FEASIBLE,
    INFEASIBLE,
    NO_SOLUTION,
    NO_INCUMBENT,
    INVALID,
    SOLVER_ERROR
};

inline const char* solveStatusToString(SolveStatus status) {
    switch (status) {
        case SolveStatus::OPTIMAL:      return "OPTIMAL";
        case SolveStatus::FEASIBLE:     return "FEASIBLE";
        case SolveStatus::INFEASIBLE:   return "INFEASIBLE";
        case SolveStatus::NO_SOLUTION:  return "NO_SOLUTION";
        case SolveStatus::NO_INCUMBENT: return "NO_INCUMBENT";
        case SolveStatus::INVALID:      return "INVALID";
        case SolveStatus::SOLVER_ERROR: return "SOLVER_ERROR";
        default:                        return "UNKNOWN";
    }
}

enum class LPSolverStatus {
    OPTIMAL,
    INFEASIBLE,
    LIMIT,
    ABANDONED,
    SOLVER_ERROR
};

inline const char* statusToString(SolverStatus status) {
    switch (status) {
        case SolverStatus::OPTIMAL:      return "OPTIMAL";
        case SolverStatus::FEASIBLE:     return "FEASIBLE";
        case SolverStatus::INFEASIBLE:   return "INFEASIBLE";
        case SolverStatus::LIMIT:        return "LIMIT";
        case SolverStatus::INVALID:      return "INVALID";
        case SolverStatus::NO_INCUMBENT: return "NO_INCUMBENT";
        case SolverStatus::ABANDONED:    return "ABANDONED";
        case SolverStatus::SOLVER_ERROR: return "SOLVER_ERROR";
        default:                         return "UNKNOWN";
    }
}

inline const char* statusToString(LPSolverStatus status) {
    switch (status) {
        case LPSolverStatus::OPTIMAL:     return "OPTIMAL";
        case LPSolverStatus::INFEASIBLE:  return "INFEASIBLE";
        case LPSolverStatus::LIMIT:       return "LIMIT";
        case LPSolverStatus::ABANDONED:   return "ABANDONED";
        case LPSolverStatus::SOLVER_ERROR:return "SOLVER_ERROR";
        default:                          return "UNKNOWN";
    }
}

// ============================================================================
// Metadatos de columna del modelo LP
// ============================================================================

/**
 * @brief Metadatos de una columna del modelo LP.
 * 
 * Asocia el índice de columna en la matriz (varIndex) con la variable
 * de decisión x_{ij} del modelo matemático (pieza i en celda j).
 * 
 * Se guardan tanto los IDs del dominio (pieceId, cellId) como los índices
 * en los vectores del Snapshot (pieceIdx, cellIdx) para acceso O(1).
 * 
 * El mapeo κ(i,j) = i·|J| + j se puede recuperar con computeKappa().
 */
struct ColumnMeta {
    int pieceId;      ///< ID de la pieza i (dominio)
    int cellId;       ///< ID de la celda j (dominio)
    int pieceIdx;     ///< Índice de i en el vector de piezas [0, n-1]
    int cellIdx;      ///< Índice de j en el vector de celdas [0, m-1]
    int varIndex;     ///< Índice de columna en la matriz LP [0, numVars-1]
    
    ColumnMeta()
        : pieceId(-1), cellId(-1), pieceIdx(-1), cellIdx(-1), varIndex(-1) {}
    
    ColumnMeta(int pid, int cid, int pidx, int cidx, int vidx)
        : pieceId(pid), cellId(cid), pieceIdx(pidx), cellIdx(cidx), varIndex(vidx) {}
    
    /**
     * @brief Calcula κ(i,j) = i·|J| + j.
     * 
     * @param numCells Número total de celdas |J|.
     * @return int Índice κ asociado a la variable x_{ij}.
     */
    int computeKappa(int numCells) const {
        return pieceIdx * numCells + cellIdx;
    }
    
    /**
     * @brief Verifica si el metadata es válido.
     */
    bool isValid() const {
        return pieceId >= 0 && cellId >= 0 && 
               pieceIdx >= 0 && cellIdx >= 0 && varIndex >= 0;
    }
};

// ============================================================================
// Configuración de solver
// ============================================================================

struct SolverConfig {
    double primalTolerance{1e-7};
    double dualTolerance{1e-7};
    int logLevel{0};
    double maxTimeSeconds{0.0};
    int maxNodes{0};
    double integerTolerance{1e-6};
    double boundTolerance{1e-8};   ///< ε_bound para comparación de cotas
    bool useHeuristics{true};
    bool useCuts{true};
    
    SolverConfig() = default;
};

// ============================================================================
// Resultado de solver entero (Cbc, B&B)
// ============================================================================

struct SolverResult {
    SolverStatus status{SolverStatus::SOLVER_ERROR};
    std::string statusMessage;
    std::unordered_map<int, int> solution;  ///< pieceId → cellId
    double objectiveValue{std::numeric_limits<double>::infinity()};
    double executionTimeSeconds{0.0};
    int nodeCount{0};
    int iterationCount{0};
    double bestLowerBound{0.0};
    
    bool hasSolution() const noexcept {
        return status == SolverStatus::OPTIMAL ||
               status == SolverStatus::FEASIBLE ||
               !solution.empty();
    }
    
    bool isValid() const noexcept {
        return status == SolverStatus::OPTIMAL || status == SolverStatus::FEASIBLE;
    }
    
    const char* statusToString() const noexcept {
        return optimization::statusToString(status);
    }
};

// ============================================================================
// Resultado de relajación LP (Clp)
// ============================================================================

struct LPSolverResult {
    LPSolverStatus status{LPSolverStatus::SOLVER_ERROR};
    bool hasSolution{false};
    double objective{std::numeric_limits<double>::infinity()};
    std::vector<double> primal;   ///< Copia profunda de primalColumnSolution
    int iterations{0};
    std::string message;          ///< Mensaje de error o debug
    
    /**
     * @brief Verifica si la solución primal es válida.
     */
    bool isValidPrimal() const {
        return hasSolution && !primal.empty();
    }
};

} // namespace optimization

#endif // OPTIMIZATION_SOLVERTYPES_H