#ifndef OPTIMIZATION_BRANCHANDBOUNDA0_H
#define OPTIMIZATION_BRANCHANDBOUNDA0_H

#include <vector>
#include <memory>
#include <queue>
#include <unordered_map>
#include <chrono>
#include <limits>
#include <string>

#include "domain/Snapshot.h"
#include "optimization/ClpRelaxationSolver.h"
#include "optimization/LpRepairHeuristic.h"
#include "optimization/SolverTypes.h" 

namespace optimization {

/**
 * @brief Resultado final del algoritmo Branch-and-Bound A0.
 */
struct A0Result {
    SolverStatus status = SolverStatus::SOLVER_ERROR;
    std::unordered_map<int, int> incumbent;   ///< Mejor solución entera (pieceId -> cellId)
    double upperBound = std::numeric_limits<double>::infinity();
    double lowerBound = 0.0;
    double gap = 0.0;
    double executionTimeSeconds = 0.0;
    
    // Estadísticas
    int nodesExplored = 0;
    int maxQueueSize = 0;
    int lpCalls = 0;
    int simplexIterations = 0;
    int prunedByBound = 0;
    int prunedByInfeasibility = 0;
    int integerSolutionsFound = 0;
    
    std::string statusMessage;
    
    /**
     * @brief Verifica si hay incumbente válido.
     */
    bool hasIncumbent() const {
        return !incumbent.empty() && upperBound < std::numeric_limits<double>::infinity();
    }
};

/**
 * @brief Nodo compacto del árbol Branch-and-Bound.
 */
struct BBNode {
    int id;                             ///< Identificador único
    int depth;                          ///< Profundidad en el árbol
    double lowerBound;                  ///< Cota inferior LB(N)
    std::vector<double> primal;         ///< Solución LP fraccionaria
    std::vector<int> fixedToOne;        ///< Columnas fijadas a 1
    std::vector<int> fixedToZero;       ///< Columnas fijadas a 0
    
    BBNode() 
        : id(-1), depth(0), lowerBound(std::numeric_limits<double>::infinity()) {}
    
    BBNode(int nodeId, int nodeDepth, double lb)
        : id(nodeId), depth(nodeDepth), lowerBound(lb) {}
    
    BBNode(BBNode&& other) noexcept = default;
    BBNode& operator=(BBNode&& other) noexcept = default;
    
    BBNode(const BBNode&) = default;
    BBNode& operator=(const BBNode&) = default;
};

/**
 * @brief Comparador best-bound para la cola OPEN (Min-Heap).
 */
struct CompareBestBound {
    bool operator()(const BBNode& a, const BBNode& b) const {
        constexpr double EPS = 1e-9;
        
        // 1. Menor LB tiene prioridad
        if (std::abs(a.lowerBound - b.lowerBound) > EPS) {
            return a.lowerBound > b.lowerBound;
        }
        
        // 2. Mayor profundidad tiene prioridad
        if (a.depth != b.depth) {
            return a.depth < b.depth;
        }
        
        // 3. Menor id tiene prioridad
        return a.id > b.id;
    }
};

/**
 * @brief Configuración del algoritmo A0.
 */
struct A0Config {
    double epsilonInt = 1e-4;       ///< Tolerancia de integralidad (más robusta)
    double epsilonBound = 1e-8;     ///< Tolerancia de comparación de cotas
    double maxTimeSeconds = 30.0;   ///< Tiempo máximo de ejecución
    bool useRepairHeuristic = true; ///< Usar heurística de reparación
    bool useWarmStartH1 = true;     ///< Inicializar con H1
    
    A0Config() = default;
};

/**
 * @brief Estrategia A0: Branch-and-Bound propio con Clp.
 */
class BranchAndBoundA0 {
public:
    BranchAndBoundA0(const domain::Snapshot& snapshot,
                     const A0Config& config = A0Config());
    
    A0Result solve();

private:
    void initializeWithHeuristics(A0Result& result);
    
    bool processNode(BBNode& node,
                     std::priority_queue<BBNode, std::vector<BBNode>, CompareBestBound>& open,
                     A0Result& result,
                     int& nodeCounter,
                     double timeLimit,
                     bool& solverError);
    
    bool isIntegerSolution(const std::vector<double>& primal, double epsilonInt) const;
    
    std::unordered_map<int, int> convertToIntegerSolution(const std::vector<double>& primal) const;
    
    int selectBranchVariable(const std::vector<double>& primal) const;
    
    bool updateIncumbent(const std::unordered_map<int, int>& candidate,
                         double cost,
                         A0Result& result);
    
    bool tryRepairHeuristic(const BBNode& node,
                            A0Result& result,
                            double timeLimitMs);
    
    double getRemainingTime(double maxTimeSeconds) const;
    
    // --- Helpers auxiliares privados ---
    double elapsedSeconds(const std::chrono::steady_clock::time_point& startTime) const;
    double computeGap(double ub, double lb) const;
    
    // Miembros privados
    const domain::Snapshot& snapshot_;
    A0Config config_;
    std::unique_ptr<ClpRelaxationSolver> clpSolver_;
    std::chrono::steady_clock::time_point startTime_;
    
    // Mapeo optimizado de columna -> (pieceId, cellId)
    std::unordered_map<int, std::pair<int, int>> columnToPair_;
};

} // namespace optimization

#endif // OPTIMIZATION_BRANCHANDBOUNDA0_H