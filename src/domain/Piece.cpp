#include "domain/Piece.h"
#include <stdexcept>

namespace domain {

// ============================================================================
// Constructor
// ============================================================================

Piece::Piece(int id, int north, int east, int south, int west,
             double cost, int initialCell)
    : id_(id),
      colors_({{north, east, south, west}}),
      cost_(cost),
      initialCell_(initialCell) {
    if (cost <= 0.0) {
        throw std::invalid_argument("El costo de la pieza debe ser positivo.");
    }
    if (initialCell < 0) {
        throw std::invalid_argument("La celda inicial no puede ser negativa.");
    }
}

// ============================================================================
// Getters
// ============================================================================

int Piece::getColor(Side side) const {
    // Mapeo directo sin validación de rango. El enum Side tiene
    // exactamente 4 valores (N=0, E=1, S=2, O=3).
    switch (side) {
        case Side::N: return colors_[0];
        case Side::E: return colors_[1];
        case Side::S: return colors_[2];
        case Side::O: return colors_[3];
    }
    throw std::runtime_error("Lado inválido en Piece::getColor");
}

// ============================================================================
// Análisis de lados grises
// ============================================================================

int Piece::countGraySides() const noexcept {
    int count = 0;
    if (colors_[0] == 0) ++count;
    if (colors_[1] == 0) ++count;
    if (colors_[2] == 0) ++count;
    if (colors_[3] == 0) ++count;
    return count;
}

/**
 * @brief Verifica la validez del patrón de lados grises.
 *
 * Implementación sin allocaciones ni bucles explícitos:
 *  1. Contar grises.
 *  2. Si <= 1: válido.
 *  3. Si == 2: verificar adyacencia con comparaciones directas.
 *  4. Si >= 3: inválido.
 *
 * El orden de los chequeos está optimizado para el caso común
 * (0, 1 o 2 grises), donde la mayoría de las piezas caen.
 */
bool Piece::hasValidGrayPattern() const noexcept {
    const int grays = countGraySides();

    // Caso 1: 0 o 1 gris -> siempre válido.
    if (grays <= 1) {
        return true;
    }

    // Caso 2: exactamente 2 grises -> verificar adyacencia.
    // Pares adyacentes válidos: (N,E), (E,S), (S,O), (O,N).
    if (grays == 2) {
        return (colors_[0] == 0 && colors_[1] == 0) ||  // N,E
               (colors_[1] == 0 && colors_[2] == 0) ||  // E,S
               (colors_[2] == 0 && colors_[3] == 0) ||  // S,O
               (colors_[3] == 0 && colors_[0] == 0);    // O,N
    }

    // Caso 3: 3 o 4 grises -> inválido.
    return false;
}

} // namespace domain