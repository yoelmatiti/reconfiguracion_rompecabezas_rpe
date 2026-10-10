#include "domain/Command.h"
#include <string>

namespace domain {

// ============================================================================
// SellarCommand
// ============================================================================

SellarCommand::SellarCommand(int cellId) : cellId_(cellId) {}

bool SellarCommand::execute(Grid& grid) const {
    try {
        grid.applySellar(cellId_);
        return true;
    } catch (const std::exception&) {
        return false;  // Transición inválida
    }
}

std::string SellarCommand::toString() const {
    return "SELLAR " + std::to_string(cellId_);
}

// ============================================================================
// PistaCommand
// ============================================================================

PistaCommand::PistaCommand(int cellId, Side side, int color)
    : cellId_(cellId), side_(side), color_(color) {}

bool PistaCommand::execute(Grid& grid) const {
    try {
        grid.applyPista(cellId_, side_, color_);
        return true;
    } catch (const std::exception&) {
        return false;  // Transición inválida
    }
}

std::string PistaCommand::toString() const {
    return "PISTA " + std::to_string(cellId_) + " "
           + sideToString(side_) + " " + std::to_string(color_);
}

// ============================================================================
// LiberarCommand
// ============================================================================

LiberarCommand::LiberarCommand(int cellId) : cellId_(cellId) {}

bool LiberarCommand::execute(Grid& grid) const {
    try {
        grid.applyLiberar(cellId_);
        return true;
    } catch (const std::exception&) {
        return false;  // Transición inválida
    }
}

std::string LiberarCommand::toString() const {
    return "LIBERAR " + std::to_string(cellId_);
}

} // namespace domain