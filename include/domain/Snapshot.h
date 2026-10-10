#ifndef DOMAIN_SNAPSHOT_H
#define DOMAIN_SNAPSHOT_H

#include <memory>
#include <vector>
#include <unordered_map>
#include <utility>
#include <cstddef>

#include "domain/Piece.h"
#include "domain/Cell.h"
#include "domain/GridGeometry.h"

namespace domain {

/**
 * @brief Fotografía inmutable del problema en una época t.
 *
 * Contiene la geometría, las celdas con sus estados actuales, las piezas,
 * la ubicación previa (t-1) y los costos de movimiento.
 */
class Snapshot {
public:
    /**
     * @brief Constructor que congela el estado de la época t.
     *
     * @param geometry Geometría del tablero (transferencia de propiedad).
     * @param pieces Vector de piezas (transferencia de propiedad).
     * @param cells Vector de celdas con su estado actual.
     * @param previousPlacement Mapa pieceId -> cellId de la época t-1.
     * @param moveCosts Mapa pieceId -> costo s_i de reubicación.
     * @throws std::invalid_argument si geometry es nulo o hay IDs duplicados.
     */
    Snapshot(std::unique_ptr<GridGeometry> geometry,
             std::vector<Piece> pieces,
             std::vector<Cell> cells,
             std::unordered_map<int, int> previousPlacement,
             std::unordered_map<int, double> moveCosts);

    // --- Consultas de componentes ---
    const GridGeometry& getGeometry() const { return *geometry_; }
    const std::vector<Piece>& getPieces() const { return pieces_; }
    const std::vector<Cell>& getCells() const { return cells_; }

    /**
     * @brief Busca una pieza por ID.
     * @return Puntero a la pieza o nullptr si no existe.
     */
    const Piece* findPiece(int pieceId) const;

    /**
     * @brief Retorna la celda que ocupaba la pieza en t-1.
     * @return cellId o -1 si no estaba colocada.
     */
    int getPreviousCell(int pieceId) const;

    /**
     * @brief Calcula el costo de colocar la pieza en la celda.
     *
     * Retorna 0 si la celda coincide con la posición en t-1, s_i en otro caso.
     */
    double getMoveCost(int pieceId, int cellId) const;

    /**
     * @brief Retorna la capacidad K_j^t de la celda.
     * @return 1 si está ABIERTA o CON_PISTA, 0 si está SELLADA.
     * @throws std::out_of_range si cellId está fuera de rango.
     */
    int getCellCapacity(int cellId) const;

    /**
     * @brief Retorna la pista vigente de la celda o nullptr.
     */
    const std::pair<Side, int>* getCellClue(int cellId) const;

private:
    std::unique_ptr<GridGeometry> geometry_;
    std::vector<Piece> pieces_;
    std::vector<Cell> cells_;
    std::unordered_map<int, int> previousPlacement_;   // pieceId -> cellId en t-1
    std::unordered_map<int, double> moveCosts_;        // pieceId -> costo s_i
    std::unordered_map<int, std::size_t> pieceIndices_; // pieceId -> índice en pieces_
};

} // namespace domain

#endif // DOMAIN_SNAPSHOT_H