#include "optimization/H1Strategy.h"
#include "optimization/HeuristicaH1.h"
#include "domain/Validator.h"

#include <chrono>
#include <stdexcept>

namespace optimization {

SolveResult H1Strategy::solve(const domain::Snapshot& snapshot,
                               const RunConfig& config) {
    SolveResult result;
    result.methodName = "H1";

    const auto startTime = std::chrono::steady_clock::now();

    try {
        // H1 no usa prevSolution aquí (se asume vacío para el estado inicial).
        // Si el Simulator quiere preservar inercia, puede inyectar prevSolution
        // vía un parámetro extendido en el futuro.
        Solution h1Sol = HeuristicH1::solve(snapshot, Solution(),
                                             config.maxTimeSeconds);

        result.executionTimeSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startTime).count();

        if (h1Sol.empty() || h1Sol.size() != snapshot.getPieces().size()) {
            result.status = SolveStatus::NO_SOLUTION;
            result.objective = std::numeric_limits<double>::infinity();
            return result;
        }

        // Validar con el Validator.
        std::unordered_map<int, int> h1Map;
        h1Map.reserve(h1Sol.size());
        for (size_t i = 0; i < h1Sol.size(); ++i) {
            if (h1Sol[i] >= 0) {
                h1Map[snapshot.getPieces()[i].getId()] = h1Sol[i];
            }
        }

        if (h1Map.empty()) {
            result.status = SolveStatus::NO_SOLUTION;
            return result;
        }

        auto val = domain::Validator::validate(h1Map, snapshot);
        if (!val.first) {
            result.status = SolveStatus::INVALID;
            return result;
        }

        result.status = SolveStatus::FEASIBLE;
        result.solution = std::move(h1Sol);
        result.objective = val.second;
        result.upperBound = val.second;

    } catch (const std::exception& e) {
        result.status = SolveStatus::SOLVER_ERROR;
        result.message = std::string("H1 exception: ") + e.what();
    }

    return result;
}

} // namespace optimization