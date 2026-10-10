#ifndef DOMAIN_GRID_H
#define DOMAIN_GRID_H

#include <vector>
#include <memory>
#include "domain/Cell.h"
#include "domain/GridGeometry.h"
#include "domain/Piece.h"
#include "domain/Side.h"
#include "domain/Snapshot.h"

namespace domain {

/**
 * @brief Contenedor principal del tablero y sus piezas.
 *
 * Gestiona las celdas, piezas y aplica comandos de estado.
 * Proporciona copia profunda para soportar rollback atómico de lotes.
 */
class Grid {
public:
    /**
     * @brief Constructor.
     */
    Grid(std::unique_ptr<GridGeometry> geometry,
         std::vector<Piece> pieces,
         std::vector<Cell> cells);

    // --- Constructores de copia y movimiento ---
    /**
     * @brief Constructor de copia (copia profunda).
     */
    Grid(const Grid& other);

    /**
     * @brief Operador de asignación por copia (copia profunda).
     */
    Grid& operator=(const Grid& other);

    /**
     * @brief Constructor de movimiento.
     */
    Grid(Grid&& other) noexcept;

    /**
     * @brief Operador de asignación por movimiento.
     */
    Grid& operator=(Grid&& other) noexcept;

    // --- Getters ---
    const GridGeometry& getGeometry() const { return *geometry_; }
    const std::vector<Piece>& getPieces() const { return pieces_; }
    const std::vector<Cell>& getCells() const { return cells_; }

    // --- Mutadores para comandos ---
    void applySellar(int cellId);
    void applyPista(int cellId, Side side, int color);
    void applyLiberar(int cellId);

    /**
     * @brief Crea un Snapshot inmutable del estado actual.
     */
    std::unique_ptr<Snapshot> createSnapshot() const;

    /**
     * @brief Realiza una copia profunda del Grid.
     *
     * Útil para aplicar lotes de comandos atómicamente con rollback:
     * se clona el grid, se aplican los comandos sobre la copia, y si
     * alguno falla, se descarta la copia dejando el original intacto.
     *
     * @return Grid Copia profunda del estado actual.
     */
    Grid deepCopy() const;

private:
    Cell& cellAt(int cellId);

    std::unique_ptr<GridGeometry> geometry_;
    std::vector<Piece> pieces_;
    std::vector<Cell> cells_;
};

} // namespace domain

#endif // DOMAIN_GRID_H