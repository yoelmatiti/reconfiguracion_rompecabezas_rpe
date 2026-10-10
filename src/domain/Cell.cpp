#include "domain/Cell.h"
#include <stdexcept>

namespace domain {

// ============================================================================
// Constructores / destructores
// ============================================================================

Cell::Cell(int id)
    : id_(id),
      state_(new OpenState()),
      clue_(Side::N, 0) {
    if (id < 0) {
        throw std::invalid_argument("El ID de la celda no puede ser negativo.");
    }
}

Cell::Cell(const Cell& other)
    : id_(other.id_),
      state_(other.state_->clone()),
      clue_(other.clue_) {
}

Cell& Cell::operator=(const Cell& other) {
    if (this != &other) {
        id_ = other.id_;
        state_ = other.state_->clone();
        clue_ = other.clue_;
    }
    return *this;
}

Cell::Cell(Cell&& other) noexcept
    : id_(other.id_),
      state_(std::move(other.state_)),
      clue_(other.clue_) {
    // Después de mover, el estado de other queda en nullptr.
    // No es necesario tocarlo, ya que other será destruido.
    // Si se llegara a usar other, getCapacity() retornaría 0 (comportamiento seguro).
}

Cell& Cell::operator=(Cell&& other) noexcept {
    if (this != &other) {
        id_ = other.id_;
        state_ = std::move(other.state_);
        clue_ = other.clue_;
    }
    return *this;
}

// ============================================================================
// Consultas
// ============================================================================

const std::pair<Side, int>* Cell::getClue() const noexcept {
    if (!state_->isClue()) {
        return nullptr;
    }
    return &clue_;
}

// ============================================================================
// Transiciones
// ============================================================================

void Cell::applySellar() {
    std::unique_ptr<CellState> newState = state_->handleSellar();
    if (newState) {
        state_ = std::move(newState);
        // Al sellar, la pista se pierde.
        clue_ = std::make_pair(Side::N, 0);
    }
}

void Cell::applyPista(Side clueSide, int color) {
    std::unique_ptr<CellState> newState = state_->handlePista(clueSide, color);
    if (newState) {
        state_ = std::move(newState);
        clue_ = std::make_pair(clueSide, color);
    }
}

void Cell::applyLiberar() {
    std::unique_ptr<CellState> newState = state_->handleLiberar();
    if (newState) {
        state_ = std::move(newState);
        clue_ = std::make_pair(Side::N, 0);
    }
}

} // namespace domain