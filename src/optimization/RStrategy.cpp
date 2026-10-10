#include "optimization/RStrategy.h"
#include "optimization/CbcReferenceSolver.h"
#include "domain/Validator.h"

#include <chrono>
#include <stdexcept>

namespace optimization {

SolveResult RStrategy::solve(const domain::Snapshot& snapshot,
                              const RunConfig& config) {
    SolveResult result;
    result.methodName = "R";

    const auto startTime = std::chrono::steady_clock::now();

    try {
        SolverConfig cbcCfg;
        cbcCfg.maxTimeSeconds   = config.maxTimeSeconds;
        cbcCfg.logLevel         = config.logLevel;
        cbcCfg.useCuts          = true;
        cbcCfg.useHeuristics    = true;
        cbcCfg.integerTolerance = config.epsilonInt;

        SolverResult cbcRes = CbcReferenceSolver::solve(snapshot, cbcCfg);

        result.executionTimeSeconds = cbcRes.executionTimeSeconds;
        result.lowerBound = cbcRes.bestLowerBound;
        result.upperBound = cbcRes.objectiveValue;
        result.nodesExplored = cbcRes.nodeCount;
        result.simplexIterations = cbcRes.iterationCount;

        // Traducir status.
        switch (cbcRes.status) {
            case SolverStatus::OPTIMAL:      result.status = SolveStatus::OPTIMAL; break;
            case SolverStatus::FEASIBLE:     result.status = SolveStatus::FEASIBLE; break;
            case SolverStatus::INFEASIBLE:   result.status = SolveStatus::INFEASIBLE; break;
            case SolverStatus::NO_INCUMBENT: result.status = SolveStatus::NO_INCUMBENT; break;
            case SolverStatus::INVALID:      result.status = SolveStatus::INVALID; break;
            case SolverStatus::SOLVER_ERROR: result.status = SolveStatus::SOLVER_ERROR; break;
            default:                         result.status = SolveStatus::SOLVER_ERROR; break;
        }

        // Convertir solution (map) a Solution (vector).
        if (cbcRes.hasSolution()) {
            const auto& pieces = snapshot.getPieces();
            result.solution.assign(pieces.size(), -1);
            for (size_t i = 0; i < pieces.size(); ++i) {
                auto it = cbcRes.solution.find(pieces[i].getId());
                if (it != cbcRes.solution.end()) {
                    result.solution[i] = it->second;
                }
            }
            result.objective = cbcRes.objectiveValue;
        } else {
            result.objective = std::numeric_limits<double>::infinity();
        }

        result.message = cbcRes.statusMessage;

    } catch (const std::exception& e) {
        result.status = SolveStatus::SOLVER_ERROR;
        result.message = std::string("R exception: ") + e.what();
        result.executionTimeSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startTime).count();
    }

    return result;
}

} // namespace optimization