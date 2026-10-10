#include "optimization/A0Strategy.h"
#include "optimization/BranchAndBoundA0.h"
#include "domain/Validator.h"

#include <chrono>
#include <stdexcept>

namespace optimization {

SolveResult A0Strategy::solve(const domain::Snapshot& snapshot,
                               const RunConfig& config) {
    SolveResult result;
    result.methodName = "A0";

    const auto startTime = std::chrono::steady_clock::now();

    try {
        // Construir y ejecutar A0.
        A0Config a0Cfg;
        a0Cfg.maxTimeSeconds    = config.maxTimeSeconds;
        a0Cfg.epsilonInt        = config.epsilonInt;
        a0Cfg.epsilonBound      = config.epsilonBound;
        a0Cfg.useRepairHeuristic= config.useRepairHeuristic;
        a0Cfg.useWarmStartH1    = config.useWarmStartH1;

        BranchAndBoundA0 a0(snapshot, a0Cfg);
        A0Result a0Res = a0.solve();

        // Traducir A0Result → SolveResult.
        result.executionTimeSeconds = a0Res.executionTimeSeconds;
        result.lowerBound = a0Res.lowerBound;
        result.upperBound = a0Res.upperBound;
        result.gapPercent = a0Res.gap;
        result.nodesExplored = a0Res.nodesExplored;
        result.lpCalls = a0Res.lpCalls;
        result.simplexIterations = a0Res.simplexIterations;
        result.prunedByBound = a0Res.prunedByBound;
        result.prunedByInfeasibility = a0Res.prunedByInfeasibility;

        // Traducir status.
        switch (a0Res.status) {
            case SolverStatus::OPTIMAL:      result.status = SolveStatus::OPTIMAL; break;
            case SolverStatus::FEASIBLE:     result.status = SolveStatus::FEASIBLE; break;
            case SolverStatus::INFEASIBLE:   result.status = SolveStatus::INFEASIBLE; break;
            case SolverStatus::NO_INCUMBENT: result.status = SolveStatus::NO_INCUMBENT; break;
            case SolverStatus::SOLVER_ERROR: result.status = SolveStatus::SOLVER_ERROR; break;
            default:                         result.status = SolveStatus::SOLVER_ERROR; break;
        }

        // Convertir incumbent (map pieceId→cellId) a Solution (vector por pieceIdx).
        if (a0Res.hasIncumbent()) {
            const auto& pieces = snapshot.getPieces();
            result.solution.assign(pieces.size(), -1);
            for (size_t i = 0; i < pieces.size(); ++i) {
                auto it = a0Res.incumbent.find(pieces[i].getId());
                if (it != a0Res.incumbent.end()) {
                    result.solution[i] = it->second;
                }
            }

            // Validar la solución antes de aceptarla.
            auto val = domain::Validator::validate(a0Res.incumbent, snapshot);
            if (val.first) {
                result.objective = val.second;
            } else {
                result.status = SolveStatus::INVALID;
                result.objective = std::numeric_limits<double>::infinity();
                result.solution.clear();
            }
        } else {
            result.objective = std::numeric_limits<double>::infinity();
        }

        result.message = a0Res.statusMessage;

    } catch (const std::exception& e) {
        result.status = SolveStatus::SOLVER_ERROR;
        result.message = std::string("A0 exception: ") + e.what();
        result.executionTimeSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startTime).count();
    }

    return result;
}

} // namespace optimization