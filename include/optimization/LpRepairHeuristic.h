#ifndef OPTIMIZATION_LPREPAIRHEURISTIC_H
#define OPTIMIZATION_LPREPAIRHEURISTIC_H

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "domain/Snapshot.h"
#include "optimization/ClpRelaxationSolver.h"
#include "optimization/SolverTypes.h"

namespace optimization {

/**
 * @brief Resultado de la heurística de reparación desde LP.
 */
struct RepairResult {
    bool success;
    std::unordered_map<int, int> solution;
    double cost;
    std::vector<int> unassigned;
    std::string errorMessage;

    RepairResult();
    RepairResult(bool success,
                 std::unordered_map<int, int> solution,
                 double cost);
};

/**
 * @brief Heurística de reparación desde solución LP (Listing 2).
 * 
 * Implementa 3 fases:
 *   1. Redondeo tentativo (max LP)
 *   2. Truncación selectiva (eliminar violadores)
 *   3. Reinserción factible (First-Fail)
 * 
 * Optimizaciones:
 *   - Mapeo inverso cellToPiece O(1)
 *   - Verificación incremental de violaciones
 *   - std::vector en lugar de std::set
 *   - Precomputación de vecinos por celda
 */
class LpRepairHeuristic {
public:
    struct Config {
        double epsilon;
        int maxIterations;
        double timeLimitMs;

        Config();
        explicit Config(const SolverConfig& solverCfg);
    };

    static RepairResult repair(
        const domain::Snapshot& snapshot,
        const std::vector<double>& lpSolution,
        const std::vector<int>& fixedToOne,
        const std::vector<int>& fixedToZero,
        const ClpRelaxationSolver& clpSolver,
        const Config& config = Config());

private:
    // ========================================================================
    // Estructuras internas
    // ========================================================================
    
    /**
     * @brief Estado interno de la reparación.
     * 
     * Mantiene el estado actualizado de la asignación parcial y permite
     * verificar violaciones de forma incremental.
     */
    struct RepairState {
        std::unordered_map<int, int> assignment;   ///< pieceId -> cellId
        std::vector<int> cellToPiece;              ///< cellId -> pieceId (-1 si libre)
        std::vector<bool> fixedToOneSet;           ///< Columnas fijadas a 1
        std::vector<bool> fixedToZeroSet;          ///< Columnas fijadas a 0
        
        int numPieces;
        int numCells;
        
        RepairState(int numPieces_, int numCells_);
        
        bool isCellOccupied(int cellId) const {
            return cellToPiece[cellId] != -1;
        }
        
        int pieceAt(int cellId) const {
            return cellToPiece[cellId];
        }
    };

    // ========================================================================
    // Fases principales
    // ========================================================================

    static bool tentativeRounding(
        const domain::Snapshot& snapshot,
        const std::vector<double>& lpSolution,
        const ClpRelaxationSolver& clpSolver,
        RepairState& state,
        std::vector<int>& pendingPieces);

    static bool selectiveTruncation(
        const domain::Snapshot& snapshot,
        const std::vector<double>& lpSolution,
        const ClpRelaxationSolver& clpSolver,
        RepairState& state,
        std::vector<int>& pendingPieces,
        double timeLimitMs);

    static bool feasibleInsertion(
        const domain::Snapshot& snapshot,
        const std::vector<double>& lpSolution,
        const ClpRelaxationSolver& clpSolver,
        RepairState& state,
        std::vector<int>& pendingPieces,
        double timeLimitMs);

    // ========================================================================
    // Helpers optimizados
    // ========================================================================

    /**
     * @brief Verifica si una pieza puede ir en una celda sin violar
     *        restricciones de vecindad. Usa cellToPiece O(1).
     */
    static bool checkNeighborMatching(
        int pieceId,
        int cellId,
        const RepairState& state,
        const domain::Snapshot& snapshot);

    /**
     * @brief Calcula celdas factibles para una pieza usando cellToPiece.
     * 
     * @param outCells Vector de salida (reutilizable, se hace clear()).
     */
    static void getFeasibleCells(
        int pieceId,
        const RepairState& state,
        const domain::Snapshot& snapshot,
        const ClpRelaxationSolver& clpSolver,
        std::vector<int>& outCells);

    /**
     * @brief Verifica si una asignación específica viola restricciones.
     * 
     * @return true si hay violación.
     */
    static bool hasLocalViolation(
        int pieceId,
        int cellId,
        const RepairState& state,
        const domain::Snapshot& snapshot);

    /**
     * @brief Encuentra el peor violador según criterio lexicográfico.
     */
    static int findWorstViolator(
        const std::vector<int>& violatingPieces,
        const RepairState& state,
        const std::vector<double>& lpSolution,
        const domain::Snapshot& snapshot,
        const ClpRelaxationSolver& clpSolver);
};

} // namespace optimization

#endif // OPTIMIZATION_LPREPAIRHEURISTIC_H