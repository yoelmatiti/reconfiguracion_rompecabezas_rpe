#include <cassert>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include "domain/Cell.h"
#include "domain/GridGeometry.h"
#include "domain/Piece.h"
#include "domain/Snapshot.h"
#include "domain/Validator.h"
#include "optimization/CbcReferenceSolver.h"

using namespace domain;
using namespace optimization;

void testCbcSolver() {
    std::cout << "Probando CbcReferenceSolver..." << std::endl;

    // Crear un problema pequeño de prueba: Tablero 2x2 (K=3, colores válidos: 0, 1, 2)
    std::unique_ptr<GridGeometry> geometry(new GridGeometry(2, 2, 3));

    // Piezas con colores válidos en [0, K-1] y patrones de lados exteriores compatibles
    std::vector<Piece> pieces;
    pieces.emplace_back(0, 0, 1, 1, 0, 10.0, 0); // Esquina N-O (Norte=0, Oeste=0)
    pieces.emplace_back(1, 0, 0, 1, 1, 15.0, 1); // Esquina N-E (Norte=0, Este=0)
    pieces.emplace_back(2, 1, 1, 0, 0, 20.0, 2); // Esquina S-O (Sur=0, Oeste=0)
    pieces.emplace_back(3, 1, 0, 0, 1, 25.0, 3); // Esquina S-E (Sur=0, Este=0)

    std::vector<Cell> cells;
    for (int i = 0; i < 4; ++i) {
        cells.emplace_back(i);
    }

    std::unordered_map<int, int> previousPlacement;
    std::unordered_map<int, double> costMatrix;
    for (size_t i = 0; i < pieces.size(); ++i) {
        previousPlacement[i] = pieces[i].getInitialCell();
        costMatrix[i] = pieces[i].getCost();
    }

    std::unique_ptr<Snapshot> snapshot(new Snapshot(
        std::move(geometry), pieces, cells, previousPlacement, costMatrix
    ));

    SolverConfig config;
    config.maxTimeSeconds = 5.0;
    config.logLevel = 0;
    config.useCuts = true;
    config.useHeuristics = true;

    auto result = CbcReferenceSolver::solve(*snapshot, config);

    std::cout << "Estado: " << result.statusToString() << std::endl;
    std::cout << "Costo objetivo: " << result.objectiveValue << std::endl;
    std::cout << "Nodos explorados: " << result.nodeCount << std::endl;
    std::cout << "Tiempo de ejecución: " << result.executionTimeSeconds << "s" << std::endl;

    assert(result.hasSolution());
    assert(result.isValid());
    assert(result.solution.size() == pieces.size());

    for (const auto& piece : pieces) {
        int pieceId = piece.getId();
        auto it = result.solution.find(pieceId);
        assert(it != result.solution.end());
        int assignedCell = it->second;
        assert(assignedCell >= 0 && assignedCell < 4);
        (void)assignedCell;
    }

    std::cout << "Prueba de Cbc completada exitosamente!" << std::endl;
}

int main() {
    try {
        testCbcSolver();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}