#include "optimization/H2Strategy.h"
#include "optimization/HeuristicaH2.h"
#include "domain/Validator.h"

#include <chrono>
#include <stdexcept>

namespace optimization {

SolveResult H2Strategy::solve(const domain::Snapshot& snapshot,
                               const RunConfig& config) {
    SolveResult result;
    result.methodName = "H2";

    const auto startTime = std::chrono::steady_clock::now();

    try {
        HeuristicH2::Params params;
        params.eta              = config.graspEta;
        params.lambda           = config.graspLambda;
        params.baseSeed         = config.baseSeed;
        params.epoch            = static_cast<uint32_t>(config.epoch);
        params.maxTimeSeconds   = config.maxTimeSeconds;
        params.maxIterations    = config.maxIterations;

        Solution h2Sol = HeuristicH2::solve(snapshot, params);

        result.executionTimeSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startTime).count();

        if (h2Sol.empty() || h2Sol.size() != snapshot.getPieces().size()) {
            result.status = SolveStatus::NO_SOLUTION;
            result.objective = std::numeric_limits<double>::infinity();
            return result;
        }

        // Validar con el Validator.
        std::unordered_map<int, int> h2Map;
        h2Map.reserve(h2Sol.size());
        for (size_t i = 0; i < h2Sol.size(); ++i) {
            if (h2Sol[i] >= 0) {
                h2Map[snapshot.getPieces()[i].getId()] = h2Sol[i];
            }
        }

        if (h2Map.empty()) {
            result.status = SolveStatus::NO_SOLUTION;
            return result;
        }

        auto val = domain::Validator::validate(h2Map, snapshot);
        if (!val.first) {
            result.status = SolveStatus::INVALID;
            return result;
        }

        result.status = SolveStatus::FEASIBLE;
        result.solution = std::move(h2Sol);
        result.objective = val.second;
        result.upperBound = val.second;

    } catch (const std::exception& e) {
        result.status = SolveStatus::SOLVER_ERROR;
        result.message = std::string("H2 exception: ") + e.what();
    }

    return result;
}

} // namespace optimization