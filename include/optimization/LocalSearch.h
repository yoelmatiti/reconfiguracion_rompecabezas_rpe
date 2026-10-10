#ifndef OPTIMIZATION_LOCALSEARCH_H
#define OPTIMIZATION_LOCALSEARCH_H

#include <vector>
#include <unordered_map>
#include <chrono>
#include "domain/Snapshot.h"
#include "domain/Piece.h"
#include "optimization/SolverTypes.h"

namespace optimization {

class LocalSearch {
public:
    struct Config {
        double maxTimeSeconds;
        bool useFirstImprovement; // Por defecto Best Improvement

        Config() : maxTimeSeconds(60.0), useFirstImprovement(false) {}
    };                                    // true es First Improvement, primer movimiento con Delta negativo

    struct NeighborMove {
        enum Type { NONE, RELOCATE, SWAP } type = NONE;
        int piece1_id = -1;
        int piece2_id = -1;
        size_t cell1_idx = 0;
        size_t cell2_idx = 0;
        double delta = 0.0; // Delta = Costo_nuevo - Costo_actual (negativo si mejora)
    };

    /**
     * Aplica la Búsqueda Local (Local Search) a la solución inicial X.
     */
    static Solution optimize(
        const Solution& initialSol,
        const domain::Snapshot& snapshot,
        const Config& config = Config()
    );

private:
    // Estructuras auxiliares para acelerar la consulta incremental
    struct GridState {
        int rows;
        int cols;
        std::vector<int> cellToPiece; // cell_idx -> piece_id (-1 si está libre)
        std::unordered_map<int, size_t> pieceToCell;
    };

    // Métodos para evaluación rápida e incremental
    static double calculateTotalCost(const GridState& state, const domain::Snapshot& snapshot);
    // Desplazamiento a una celda libre
    static double evaluateRelocateDelta(int pieceId, size_t fromCell, size_t toCell, const GridState& state, const domain::Snapshot& snapshot);
    // Intercambio de dos piezas
    static double evaluateSwapDelta(int p1, size_t c1, int p2, size_t c2, const GridState& state, const domain::Snapshot& snapshot);
    static double calculateCellConflicts(int pieceId, size_t cellIdx, const GridState& state, const domain::Snapshot& snapshot);

    static void applyMove(GridState& state, const NeighborMove& move);
};

} // namespace optimization

#endif // OPTIMIZATION_LOCALSEARCH_H