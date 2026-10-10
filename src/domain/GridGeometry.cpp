#include "domain/GridGeometry.h"
#include <stdexcept>

namespace domain {

// ============================================================================
// Constructor
// ============================================================================

GridGeometry::GridGeometry(int rows, int cols, int numColors)
    : rows_(rows),
      cols_(cols),
      numColors_(numColors),
      numCells_(rows * cols) {
    if (rows <= 0 || cols <= 0) {
        throw std::invalid_argument(
            "El número de filas y columnas debe ser positivo.");
    }
    if (numColors < 2) {
        throw std::invalid_argument(
            "Se requieren al menos 2 colores (0 y colores positivos).");
    }
}

// ============================================================================
// Consultas topológicas
// ============================================================================

/**
 * @brief Retorna los lados exteriores de la celda.
 *
 * El orden de inserción es N, E, S, O. Para celdas interiores, count=0.
 * Para celdas de borde, count=1. Para esquinas, count=2.
 *
 * Pre-condición: cellId ∈ [0, numCells_). No se valida internamente
 * para mantener el método noexcept y sin coste en bucles calientes.
 */
OuterSides GridGeometry::getOuterSides(int cellId) const noexcept {
    const int r = getRow(cellId);
    const int c = getCol(cellId);

    OuterSides result;

    if (r == 0)         result.sides[result.count++] = Side::N;
    if (c == cols_ - 1) result.sides[result.count++] = Side::E;
    if (r == rows_ - 1) result.sides[result.count++] = Side::S;
    if (c == 0)         result.sides[result.count++] = Side::O;

    return result;
}

/**
 * @brief Retorna las celdas adyacentes dentro del tablero.
 *
 * El orden de inserción es N, E, S, O. Una celda interior tiene 4 vecinos;
 * una de borde, 3; una esquina, 2.
 *
 * Pre-condición: cellId ∈ [0, numCells_).
 */
AdjacentCells GridGeometry::getAdjacentCells(int cellId) const noexcept {
    const int r = getRow(cellId);
    const int c = getCol(cellId);

    AdjacentCells result;

    if (r > 0) {
        result.cells[result.count].first  = getCellId(r - 1, c);
        result.cells[result.count].second = Side::N;
        ++result.count;
    }
    if (c < cols_ - 1) {
        result.cells[result.count].first  = getCellId(r, c + 1);
        result.cells[result.count].second = Side::E;
        ++result.count;
    }
    if (r < rows_ - 1) {
        result.cells[result.count].first  = getCellId(r + 1, c);
        result.cells[result.count].second = Side::S;
        ++result.count;
    }
    if (c > 0) {
        result.cells[result.count].first  = getCellId(r, c - 1);
        result.cells[result.count].second = Side::O;
        ++result.count;
    }

    return result;
}

} // namespace domain