#include "domain/CellState.h"
#include <stdexcept>
#include <string>

namespace domain {

// ============================================================================
// OpenState
// ============================================================================

std::unique_ptr<CellState> OpenState::handleSellar() const {
    return std::unique_ptr<CellState>(new SealedState());
}

std::unique_ptr<CellState> OpenState::handlePista(Side side, int color) const {
    if (color < 0) {
        throw std::invalid_argument(
            "El color de la pista no puede ser negativo.");
    }
    return std::unique_ptr<CellState>(new ClueState(side, color));
}

std::unique_ptr<CellState> OpenState::handleLiberar() const {
    throw std::logic_error(
        "Transición inválida: no se puede LIBERAR una celda ABIERTA.");
}

std::unique_ptr<CellState> OpenState::clone() const {
    return std::unique_ptr<CellState>(new OpenState());
}

// ============================================================================
// ClueState
// ============================================================================

ClueState::ClueState(Side side, int color)
    : clueSide_(side), clueColor_(color) {
    if (color < 0) {
        throw std::invalid_argument(
            "El color de la pista no puede ser negativo.");
    }
}

std::unique_ptr<CellState> ClueState::handleSellar() const {
    return std::unique_ptr<CellState>(new SealedState());
}

std::unique_ptr<CellState> ClueState::handlePista(Side side, int color) const {
    if (color < 0) {
        throw std::invalid_argument(
            "El color de la pista no puede ser negativo.");
    }
    return std::unique_ptr<CellState>(new ClueState(side, color));
}

std::unique_ptr<CellState> ClueState::handleLiberar() const {
    return std::unique_ptr<CellState>(new OpenState());
}

std::unique_ptr<CellState> ClueState::clone() const {
    return std::unique_ptr<CellState>(new ClueState(clueSide_, clueColor_));
}

// ============================================================================
// SealedState
// ============================================================================

std::unique_ptr<CellState> SealedState::handleSellar() const {
    throw std::logic_error(
        "Transición inválida: no se puede SELLAR una celda ya SELLADA.");
}

std::unique_ptr<CellState> SealedState::handlePista(Side /*side*/, int /*color*/) const {
    throw std::logic_error(
        "Transición inválida: no se puede aplicar PISTA a una celda SELLADA.");
}

std::unique_ptr<CellState> SealedState::handleLiberar() const {
    return std::unique_ptr<CellState>(new OpenState());
}

std::unique_ptr<CellState> SealedState::clone() const {
    return std::unique_ptr<CellState>(new SealedState());
}

} // namespace domain