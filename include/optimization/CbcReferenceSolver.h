#ifndef OPTIMIZATION_CBCREFERENCESOLVER_H
#define OPTIMIZATION_CBCREFERENCESOLVER_H

#include "domain/Snapshot.h"
#include "optimization/MatrixBuilder.h"
#include "optimization/SolverTypes.h"

namespace optimization {

/**
 * @brief Solucionador de referencia exacto basado en COIN-OR Cbc.
 */
class CbcReferenceSolver {
public:
    CbcReferenceSolver() = default;
    ~CbcReferenceSolver() = default;

    /**
     * @brief Resuelve el modelo lineal entero completo utilizando CbcModel.
     */
    static SolverResult solve(const domain::Snapshot& snapshot, 
                             const SolverConfig& config = SolverConfig());
};

} // namespace optimization

#endif // OPTIMIZATION_CBCREFERENCESOLVER_H