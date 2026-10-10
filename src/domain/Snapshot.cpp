#include "domain/Snapshot.h"
#include <stdexcept>

namespace domain {

// ============================================================================
// Constructor
// ============================================================================

/**
 * @brief Constructor de la fotografía del estado.
 *
 * Usa std::move para transferir la propiedad de los recursos sin copias
 * profundas. Después del constructor, los parámetros locales quedan en
 * estado "movido desde" y sus recursos están bajo propiedad del Snapshot.
 */
Snapshot::Snapshot(std::unique_ptr<GridGeometry> geometry,
                   std::vector<Piece> pieces,
                   std::vector<Cell> cells,
                   std::unordered_map<int, int> previousPlacement,
                   std::unordered_map<int, double> costMatrix)
    : geometry_(std::move(geometry)),
      pieces_(std::move(pieces)),
      cells_(std::move(cells)),
      previousPlacement_(std::move(previousPlacement)),
      moveCosts_(std::move(costMatrix)) {

    if (!geometry_) {
        throw std::invalid_argument(
            "GridGeometry no puede ser nulo en el Snapshot.");
    }

    // Reserva y verificación de unicidad de IDs.
    pieceIndices_.reserve(pieces_.size());
    for (std::size_t index = 0; index < pieces_.size(); ++index) {
        if (!pieceIndices_.emplace(pieces_[index].getId(), index).second) {
            throw std::invalid_argument(
                "No puede haber IDs de pieza duplicados.");
        }
    }
}

// ============================================================================
// Consultas
// ============================================================================

const Piece* Snapshot::findPiece(int pieceId) const {
    const auto it = pieceIndices_.find(pieceId);
    return it == pieceIndices_.end() ? nullptr : &pieces_[it->second];
}

/**
 * @brief Retorna la celda que ocupaba la pieza en t-1.
 *
 * Retorna -1 si la pieza no estaba colocada en t-1 o si su ID no existe.
 */
int Snapshot::getPreviousCell(int pieceId) const {
    const auto it = previousPlacement_.find(pieceId);
    return (it != previousPlacement_.end()) ? it->second : -1;
}

/**
 * @brief Calcula el costo de asignar la pieza a la celda.
 *
 * Optimización: fusiona el lookup de previousPlacement_ y moveCosts_
 * en una sola pasada. En bucles calientes (B&B, H1, H2, LocalSearch),
 * esto ahorra una llamada a función y un segundo lookup por invocación.
 */
double Snapshot::getMoveCost(int pieceId, int cellId) const {
    // 1. Verificar si la celda es la misma que en t-1 (costo cero).
    const auto prevIt = previousPlacement_.find(pieceId);
    if (prevIt != previousPlacement_.end() && prevIt->second == cellId) {
        return 0.0;
    }

    // 2. Consultar el costo de reubicación s_i.
    const auto costIt = moveCosts_.find(pieceId);
    if (costIt == moveCosts_.end()) {
        throw std::out_of_range(
            "ID de pieza no encontrado en la matriz de costos.");
    }
    return costIt->second;
}

int Snapshot::getCellCapacity(int cellId) const {
    if (cellId < 0 || cellId >= static_cast<int>(cells_.size())) {
        throw std::out_of_range("ID de celda fuera de rango en Snapshot.");
    }
    return cells_[cellId].getCapacity();
}

const std::pair<Side, int>* Snapshot::getCellClue(int cellId) const {
    if (cellId < 0 || cellId >= static_cast<int>(cells_.size())) {
        throw std::out_of_range("ID de celda fuera de rango en Snapshot.");
    }
    return cells_[cellId].getClue();
}

} // namespace domain