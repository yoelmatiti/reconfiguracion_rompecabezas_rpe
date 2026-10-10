#include "optimization/MatrixBuilder.h"
#include "domain/Validator.h"

#include <coin/CoinFinite.hpp>
#include <coin/CoinPackedMatrix.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace optimization {

namespace {

const double DENSITY_ESTIMATE = 0.2;

struct Adjacency {
    int cellA;
    int cellB;
    domain::Side sideA;
    domain::Side sideB;
};

} // namespace

OptimizationModelData MatrixBuilder::buildModelData(const domain::Snapshot& snapshot) {
    OptimizationModelData data;

    const auto& pieces = snapshot.getPieces();
    const auto& cells = snapshot.getCells();
    const auto& geometry = snapshot.getGeometry();

    data.numPieces = static_cast<int>(pieces.size());
    data.numCells = static_cast<int>(cells.size());

    // =========================================================================
    // PASO 1: GENERACIÓN DE VARIABLES (COLUMNAS) Y PRE-FILTRO DE COMPATIBILIDAD
    // =========================================================================
    // Reserva espacio inicial estimado para evitar reallocs continuos en memoria Heap
    const size_t estimatedVars = static_cast<size_t>(
        static_cast<double>(data.numPieces) *
        static_cast<double>(data.numCells) * DENSITY_ESTIMATE
    ) + 1;

    data.colToPairMap.reserve(estimatedVars);
    data.objective.reserve(estimatedVars);

    // Iteración sobre todos los pares candidato (pieza i, celda j)
    for (int i = 0; i < data.numPieces; ++i) {
        const auto& piece = pieces[i];
        const int pieceId = piece.getId();

        for (int j = 0; j < data.numCells; ++j) {
            const int cellId = cells[j].getId();

            // Filtro 1 (Ecuación 3d): Excluye parejas donde bordes grises no coinciden con los bordes exteriores (G_i == O_j)
            if (!isStructurallyCompatible(snapshot, i, j)) {
                continue;
            }

            // Filtro 2 (Ecuación 3c): Excluye celdas selladas en esta época (K_j^t = 0)
            if (snapshot.getCellCapacity(cellId) == 0) {
                continue;
            }

            // Instanciación de la columna x_{ij} en la matriz LP
            const int colIdx = static_cast<int>(data.colToPairMap.size());
            data.colToPairMap.emplace_back(pieceId, cellId, i, j, colIdx);
            
            // Asignación del coeficiente c_{ij}^t en la función objetivo (Costo de movimiento)
            data.objective.push_back(snapshot.getMoveCost(pieceId, cellId));
        }
    }

    data.numVars = static_cast<int>(data.colToPairMap.size());

    // Si no se generaron variables factibles, el modelo se retorna sin columnas
    if (data.numVars == 0) {
        return data;
    }

    const int numColors = geometry.getNumColors();
    const int numMatchingColors = std::max(1, numColors - 1);

    // =========================================================================
    // PASO 2: IDENTIFICACIÓN DE ADYACENCIAS (VECINDAD ENTRE CELDAS)
    // =========================================================================
    // Mapeo de pares adyacentes únicos (cellA < cellB) para construir el grafo de encajes
    std::vector<Adjacency> adjacencies;
    adjacencies.reserve(static_cast<size_t>(data.numCells) * 2);

    for (int j = 0; j < data.numCells; ++j) {
        const auto adjacentCells = geometry.getAdjacentCells(j);
        for (int idx = 0; idx < adjacentCells.count; ++idx) {
            const int neighborCell = adjacentCells.cells[idx].first;
            const domain::Side sideFromJ = adjacentCells.cells[idx].second;

            // Garantiza procesar cada arista exactamente una vez evitando duplicados
            if (j < neighborCell) {
                Adjacency adjacency;
                adjacency.cellA = j;
                adjacency.cellB = neighborCell;
                adjacency.sideA = sideFromJ;
                adjacency.sideB = oppositeSide(sideFromJ);
                adjacencies.push_back(adjacency);
            }
        }
    }

    const int numAdjacencies = static_cast<int>(adjacencies.size());

    // Índice de acceso directo en O(1) para consultar adyacencia según el par de celdas
    std::vector<int> adjIndexByPair;
    if (numAdjacencies > 0) {
        const size_t maxKey = static_cast<size_t>(data.numCells) * static_cast<size_t>(data.numCells);
        adjIndexByPair.assign(maxKey, -1);

        for (int a = 0; a < numAdjacencies; ++a) {
            const size_t key = static_cast<size_t>(adjacencies[a].cellA) * static_cast<size_t>(data.numCells) +
                               static_cast<size_t>(adjacencies[a].cellB);
            adjIndexByPair[key] = a;
        }
    }

    // =========================================================================
    // PASO 3: CÁLCULO DEL NÚMERO TOTAL DE FILAS (RESTRICCIONES DEL MODELO)
    // =========================================================================
    // numRows = (Filas de Piezas: Asignación Única 3b) 
    //         + (Filas de Celdas: Capacidad 3c) 
    //         + (Filas de Encaje de Color: Linealización 3f y 3g)
    data.numRows = data.numPieces + data.numCells + numAdjacencies * numMatchingColors * 2;

    // =========================================================================
    // PASO 4: CONSTRUCCIÓN DE LAS ENTRADAS DE LA MATRIZ DISPERSA (COEFICIENTES)
    // =========================================================================
    std::vector<int> rowIndices;
    std::vector<int> colIndices;
    std::vector<double> values;

    const size_t maxNonZerosPerColumn = 2u +
        static_cast<size_t>(2 * std::max(1, data.numCells > 0 ? data.numCells : 1) * numMatchingColors);

    rowIndices.reserve(static_cast<size_t>(data.numVars) * maxNonZerosPerColumn);
    colIndices.reserve(static_cast<size_t>(data.numVars) * maxNonZerosPerColumn);
    values.reserve(static_cast<size_t>(data.numVars) * maxNonZerosPerColumn);

    // Mapea las variables x_{ij} creadas a sus posiciones correspondientes en las filas del modelo
    for (int colIdx = 0; colIdx < data.numVars; ++colIdx) {
        const ColumnMeta& meta = data.colToPairMap[colIdx];
        const int pieceIdx = meta.pieceIdx;
        const int cellIdx = meta.cellIdx;

        // 1. Coeficiente en restricción de Asignación Única: Fila = pieceIdx
        rowIndices.push_back(pieceIdx);
        colIndices.push_back(colIdx);
        values.push_back(1.0);

        // 2. Coeficiente en restricción de Capacidad: Fila = numPieces + cellIdx
        const int capacityRow = data.numPieces + cellIdx;
        rowIndices.push_back(capacityRow);
        colIndices.push_back(colIdx);
        values.push_back(1.0);

        // 3. Coeficientes en restricciones de Encaje de Colores entre Vecinas (3f–3g)
        const auto adjacentCells = geometry.getAdjacentCells(cellIdx);
        for (int idx = 0; idx < adjacentCells.count; ++idx) {
            const int neighborCell = adjacentCells.cells[idx].first;
            const domain::Side mySide = adjacentCells.cells[idx].second;

            const int cellA = std::min(cellIdx, neighborCell);
            const int cellB = std::max(cellIdx, neighborCell);

            if (cellA == cellB) {
                continue;
            }

            const size_t key = static_cast<size_t>(cellA) * static_cast<size_t>(data.numCells) +
                               static_cast<size_t>(cellB);

            if (key >= adjIndexByPair.size() || adjIndexByPair[key] < 0) {
                continue;
            }

            const int adjIdx = adjIndexByPair[key];
            const bool isCellA = (cellIdx == cellA);
            const int pieceColor = pieces[pieceIdx].getColor(mySide);

            for (int color = 1; color < numColors; ++color) {
                const int colorOffset = color - 1;
                const int baseRow = data.numPieces + data.numCells +
                    (adjIdx * numMatchingColors + colorOffset) * 2;

                const int row3f = baseRow;
                const int row3g = baseRow + 1;

                const double indicator = (pieceColor == color) ? 1.0 : 0.0;

                double coef3f = 0.0;
                double coef3g = 0.0;

                if (isCellA) {
                    coef3f = 1.0 + indicator;
                    coef3g = 1.0 - indicator;
                } else {
                    coef3f = 1.0 - indicator;
                    coef3g = 1.0 + indicator;
                }

                if (std::abs(coef3f) > 1e-12) {
                    rowIndices.push_back(row3f);
                    colIndices.push_back(colIdx);
                    values.push_back(coef3f);
                }

                if (std::abs(coef3g) > 1e-12) {
                    rowIndices.push_back(row3g);
                    colIndices.push_back(colIdx);
                    values.push_back(coef3g);
                }
            }
        }
    }

    // =========================================================================
    // PASO 5: INSTANCIACIÓN DE COINPACKEDMATRIX Y COTAS DE FILAS/COLUMNAS
    // =========================================================================
    // Empaquetado final de los datos en la estructura de matriz dispersa de COIN-OR
    data.matrix = std::unique_ptr<CoinPackedMatrix>(
        new CoinPackedMatrix(
            true,
            rowIndices.data(),
            colIndices.data(),
            values.data(),
            static_cast<CoinBigIndex>(values.size())
        )
    );

    if (!data.matrix) {
        throw std::runtime_error("Error al construir la matriz dispersa");
    }

    // Cotas primarias de variables binarias (Relajación Continua): 0.0 <= x_{ij} <= 1.0
    data.colLowerBounds.assign(data.numVars, 0.0);
    data.colUpperBounds.assign(data.numVars, 1.0);

    // Inicialización de vectores de cotas del lado derecho (RHS) para cada fila
    data.rowLowerBounds.assign(data.numRows, 0.0);
    data.rowUpperBounds.assign(data.numRows, 0.0);

    // Cotas Fila Bloque 1 (Ecuación 3b): Igualdad estricta \sum_j x_{ij} = 1.0
    for (int i = 0; i < data.numPieces; ++i) {
        data.rowLowerBounds[i] = 1.0;
        data.rowUpperBounds[i] = 1.0;
    }

    // Cotas Fila Bloque 2 (Ecuación 3c): Capacidad de celda 0.0 <= \sum_i x_{ij} <= K_j^t
    for (int j = 0; j < data.numCells; ++j) {
        const int rowIdx = data.numPieces + j;
        data.rowLowerBounds[rowIdx] = 0.0;
        data.rowUpperBounds[rowIdx] = static_cast<double>(snapshot.getCellCapacity(j));
    }

    // Cotas Fila Bloque 3 (Ecuaciones 3f y 3g): Desigualdad de encaje -\infty <= Fila <= 2.0
    for (int rowIdx = data.numPieces + data.numCells; rowIdx < data.numRows; ++rowIdx) {
        data.rowLowerBounds[rowIdx] = -COIN_DBL_MAX;
        data.rowUpperBounds[rowIdx] = 2.0;
    }

    // =========================================================================
    // PASO 6: APLICACIÓN DE PISTAS VIGENTES (ECUACIÓN 3e)
    // =========================================================================
    // Inhabilita fijando cotas a [0.0, 0.0] aquellas variables que violan pistas activas
    for (int colIdx = 0; colIdx < data.numVars; ++colIdx) {
        const ColumnMeta& meta = data.colToPairMap[colIdx];
        if (isProhibitedByClue(snapshot, meta.pieceIdx, meta.cellIdx)) {
            data.colLowerBounds[colIdx] = 0.0;
            data.colUpperBounds[colIdx] = 0.0;
        }
    }

    // Aseveraciones para verificar consistencia dimensional
    assert(data.colToPairMap.size() == static_cast<size_t>(data.numVars));
    assert(data.objective.size() == static_cast<size_t>(data.numVars));
    assert(data.colLowerBounds.size() == static_cast<size_t>(data.numVars));
    assert(data.colUpperBounds.size() == static_cast<size_t>(data.numVars));
    assert(data.rowLowerBounds.size() == static_cast<size_t>(data.numRows));
    assert(data.rowUpperBounds.size() == static_cast<size_t>(data.numRows));
    assert(data.matrix->getNumCols() == data.numVars);
    assert(data.matrix->getNumRows() == data.numRows);

    return data;
}

bool MatrixBuilder::isStructurallyCompatible(
    const domain::Snapshot& snapshot,
    int pieceIdx,
    int cellIdx) {

    const auto& pieces = snapshot.getPieces();
    const auto& geometry = snapshot.getGeometry();

    const domain::OuterSides outerSides = geometry.getOuterSides(cellIdx);
    return domain::Validator::checkStructuralCompatibility(pieces[pieceIdx], outerSides);
}

bool MatrixBuilder::isProhibitedByClue(
    const domain::Snapshot& snapshot,
    int pieceIdx,
    int cellIdx) {

    const auto& pieces = snapshot.getPieces();
    const auto& cells = snapshot.getCells();

    const auto& cell = cells[cellIdx];
    if (!cell.hasClue()) {
        return false;
    }

    const auto* clue = cell.getClue();
    if (clue == nullptr) {
        return false;
    }

    const domain::Side clueSide = clue->first;
    const int clueColor = clue->second;

    const auto& piece = pieces[pieceIdx];
    const int pieceColor = piece.getColor(clueSide);

    return (pieceColor != clueColor);
}

int MatrixBuilder::computeKappa(int pieceIdx, int cellIdx, int numCells) {
    return pieceIdx * numCells + cellIdx;
}

domain::Side MatrixBuilder::oppositeSide(domain::Side side) {
    switch (side) {
        case domain::Side::N: return domain::Side::S;
        case domain::Side::E: return domain::Side::O;
        case domain::Side::S: return domain::Side::N;
        case domain::Side::O: return domain::Side::E;
    }
    throw std::runtime_error("Lado inválido");
}

} // namespace optimization