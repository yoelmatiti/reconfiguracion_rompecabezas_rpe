#ifndef OPTIMIZATION_H1STRATEGY_H
#define OPTIMIZATION_H1STRATEGY_H

#include "optimization/PlacementStrategy.h"

namespace optimization {

/**
 * @brief Adaptador de HeuristicH1 al Patrón Strategy.
 */
class H1Strategy : public PlacementStrategy {
public:
    H1Strategy() = default;
    ~H1Strategy() override = default;

    SolveResult solve(const domain::Snapshot& snapshot,
                      const RunConfig& config) override;

    const char* name() const override { return "H1"; }
};

} // namespace optimization

#endif // OPTIMIZATION_H1STRATEGY_H