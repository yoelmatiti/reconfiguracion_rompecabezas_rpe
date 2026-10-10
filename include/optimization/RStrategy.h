#ifndef OPTIMIZATION_RSTRATEGY_H
#define OPTIMIZATION_RSTRATEGY_H

#include "optimization/PlacementStrategy.h"

namespace optimization {

/**
 * @brief Adaptador de CbcReferenceSolver al Patrón Strategy.
 */
class RStrategy : public PlacementStrategy {
public:
    RStrategy() = default;
    ~RStrategy() override = default;

    SolveResult solve(const domain::Snapshot& snapshot,
                      const RunConfig& config) override;

    const char* name() const override { return "R"; }
};

} // namespace optimization

#endif // OPTIMIZATION_RSTRATEGY_H