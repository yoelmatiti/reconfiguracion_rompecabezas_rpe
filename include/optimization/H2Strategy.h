#ifndef OPTIMIZATION_H2STRATEGY_H
#define OPTIMIZATION_H2STRATEGY_H

#include "optimization/PlacementStrategy.h"

namespace optimization {

/**
 * @brief Adaptador de HeuristicH2 (GRASP) al Patrón Strategy.
 */
class H2Strategy : public PlacementStrategy {
public:
    H2Strategy() = default;
    ~H2Strategy() override = default;

    SolveResult solve(const domain::Snapshot& snapshot,
                      const RunConfig& config) override;

    const char* name() const override { return "H2"; }
};

} // namespace optimization

#endif // OPTIMIZATION_H2STRATEGY_H