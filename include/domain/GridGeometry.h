#ifndef DOMAIN_GRIDGEOMETRY_H
#define DOMAIN_GRIDGEOMETRY_H

#include <array>
#include <utility>
#include "domain/Side.h"

namespace domain {

/**
 * @brief Estructura ultra-rápida para devolver vecinos sin usar el Heap.
 *
 * Almacena hasta 4 vecinos (N, E, S, O) en un array fijo. Cero allocaciones.
 */
struct AdjacentCells {
    std::array<std::pair<int, Side>, 4> cells;
    int count = 0;  // Número de vecinos reales (2, 3 o 4)
};

/**
 * @brief Estructura para lados exteriores (0 allocaciones de Heap).
 *
 * Almacena hasta 4 lados exteriores de una celda. El orden interno
 * coincide con la convención N, E, S, O, pero el llamador no debe
 * asumir ningún orden específico.
 */
struct OuterSides {
    std::array<Side, 4> sides;
    int count = 0;

    /**
     * @brief Verifica si un lado específico está entre los exteriores.
     *
     * Costo: O(1) (bucle de tamaño máximo 2 en la práctica, ya que
     * una celda tiene 0, 1 o 2 lados exteriores).
     *
     * @param side Lado a verificar.
     * @return true si el lado es exterior.
     */
    inline bool contains(Side side) const noexcept {
        for (int i = 0; i < count; ++i) {
            if (sides[i] == side) return true;
        }
        return false;
    }
};

/**
 * @brief Clase inmutable que encapsula la topología fija y estática del tablero.
 *
 * La geometría (número de filas, columnas, colores) no cambia entre épocas.
 * Se construye una vez a partir del bloque BOARD del archivo .rpe.
 */
class GridGeometry {
public:
    /**
     * @brief Constructor.
     *
     * @param rows Número de filas (>= 1).
     * @param cols Número de columnas (>= 1).
     * @param numColors Número de colores en la paleta (>= 2, incluyendo 0).
     * @throws std::invalid_argument si algún parámetro es inválido.
     */
    GridGeometry(int rows, int cols, int numColors);

    // --- Consultas O(1) ---
    int getRows() const noexcept { return rows_; }
    int getCols() const noexcept { return cols_; }
    int getNumCells() const noexcept { return numCells_; }
    int getNumColors() const noexcept { return numColors_; }

    /**
     * @brief Retorna los lados exteriores de una celda.
     *
     * @param cellId Índice de la celda.
     * @pre cellId ∈ [0, numCells_). El llamador debe garantizar este rango.
     * @return OuterSides Estructura con los lados exteriores (count en [0, 2]).
     *
     * @note No valida el rango para mantener noexcept y costo cero. El llamador
     *       (parser, solver) garantiza la pre-condición.
     */
    OuterSides getOuterSides(int cellId) const noexcept;

    /**
     * @brief Retorna las celdas adyacentes y el lado compartido.
     *
     * @param cellId Índice de la celda.
     * @pre cellId ∈ [0, numCells_).
     * @return AdjacentCells Estructura con vecinos dentro del tablero.
     *
     * @note El orden es N, E, S, O (solo los que existen).
     */
    AdjacentCells getAdjacentCells(int cellId) const noexcept;

    // --- Conversión de coordenadas (O(1)) ---
    inline int getRow(int cellId) const noexcept { return cellId / cols_; }
    inline int getCol(int cellId) const noexcept { return cellId % cols_; }
    inline int getCellId(int row, int col) const noexcept {
        return row * cols_ + col;
    }

private:
    int rows_;
    int cols_;
    int numColors_;
    int numCells_;  // Caché de R · C
};

} // namespace domain

#endif // DOMAIN_GRIDGEOMETRY_H