#include "domain/Grid.h"
#include "domain/Snapshot.h"
#include <stdexcept>
#include <unordered_map>

namespace domain {

// ============================================================================
// Constructores
// ============================================================================

Grid::Grid(std::unique_ptr<GridGeometry> geometry,
           std::vector<Piece> pieces,
           std::vector<Cell> cells)
    : geometry_(std::move(geometry)),
      pieces_(std::move(pieces)),
      cells_(std::move(cells)) {
    if (!geometry_) {
        throw std::invalid_argument("GridGeometry no puede ser nulo.");
    }
    if (cells_.size() != static_cast<std::size_t>(geometry_->getNumCells())) {
        throw std::invalid_argument(
            "La cantidad de celdas no coincide con la geometría.");
    }
    for (std::size_t index = 0; index < cells_.size(); ++index) {
        if (cells_[index].getId() != static_cast<int>(index)) {
            throw std::invalid_argument(
                "Los IDs de celda deben coincidir con su posición.");
        }
    }
}

// Copia profunda
Grid::Grid(const Grid& other)
    : geometry_(other.geometry_
                    ? std::unique_ptr<GridGeometry>(
                          new GridGeometry(*other.geometry_))
                    : nullptr),
      pieces_(other.pieces_),   // Piece es copiable (solo PODs)
      cells_(other.cells_) {    // Cell tiene constructor de copia (patrón State)
}

Grid& Grid::operator=(const Grid& other) {
    if (this != &other) {
        geometry_ = other.geometry_
                        ? std::unique_ptr<GridGeometry>(
                              new GridGeometry(*other.geometry_))
                        : nullptr;
        pieces_ = other.pieces_;
        cells_ = other.cells_;
    }
    return *this;
}

Grid::Grid(Grid&& other) noexcept
    : geometry_(std::move(other.geometry_)),
      pieces_(std::move(other.pieces_)),
      cells_(std::move(other.cells_)) {
}

Grid& Grid::operator=(Grid&& other) noexcept {
    if (this != &other) {
        geometry_ = std::move(other.geometry_);
        pieces_ = std::move(other.pieces_);
        cells_ = std::move(other.cells_);
    }
    return *this;
}

// ============================================================================
// Mutadores
// ============================================================================

void Grid::applySellar(int cellId) {
    cellAt(cellId).applySellar();
}

void Grid::applyPista(int cellId, Side side, int color) {
    // La validación geométrica de lados exteriores se mantiene en el parser
    // para los archivos RPE, pero la API de comandos debe aceptar cualquier
    // pista que la lógica del dominio considere válida para la celda.
    cellAt(cellId).applyPista(side, color);
}

void Grid::applyLiberar(int cellId) {
    cellAt(cellId).applyLiberar();
}

Cell& Grid::cellAt(int cellId) {
    if (cellId < 0 || cellId >= static_cast<int>(cells_.size())) {
        throw std::out_of_range("ID de celda fuera de rango.");
    }
    return cells_[cellId];
}

// ============================================================================
// Snapshot
// ============================================================================

std::unique_ptr<Snapshot> Grid::createSnapshot() const {
    std::unordered_map<int, int> previousPlacement;
    std::unordered_map<int, double> costMatrix;

    for (const auto& piece : pieces_) {
        previousPlacement[piece.getId()] = piece.getInitialCell();
        costMatrix[piece.getId()] = piece.getCost();
    }

    return std::unique_ptr<Snapshot>(new Snapshot(
        std::unique_ptr<GridGeometry>(new GridGeometry(*geometry_)),
        pieces_,
        cells_,
        std::move(previousPlacement),
        std::move(costMatrix)
    ));
}

// ============================================================================
// Copia profunda explícita
// ============================================================================

Grid Grid::deepCopy() const {
    return Grid(*this);  // Invoca el constructor de copia
}

} // namespace domain