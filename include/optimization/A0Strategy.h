#ifndef OPTIMIZATION_A0STRATEGY_H
#define OPTIMIZATION_A0STRATEGY_H

#include "optimization/PlacementStrategy.h"

namespace optimization {

/**
 * @brief Adaptador de BranchAndBoundA0 al Patrón Strategy.
 *
 * Encapsula la construcción y ejecución de A0, traduciendo el A0Result
 * al SolveResult unificado.
 */
class A0Strategy : public PlacementStrategy {
public:
    A0Strategy() = default;
    ~A0Strategy() override = default;

    SolveResult solve(const domain::Snapshot& snapshot,
                      const RunConfig& config) override;

    const char* name() const override { return "A0"; }
};

} // namespace optimization

#endif // OPTIMIZATION_A0STRATEGY_H