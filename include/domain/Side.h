#ifndef DOMAIN_SIDE_H
#define DOMAIN_SIDE_H

#include <array>

namespace domain {

/**
 * @brief Enumeración para los cuatro lados de una pieza o celda.
 */
enum class Side {
    N, // Norte (Arriba)
    E, // Este (Derecha)
    S, // Sur (Abajo)
    O  // Oeste (Izquierda)
};

/// Arreglo auxiliar para iterar fácilmente: for (Side side : ALL_SIDES)
constexpr std::array<Side, 4> ALL_SIDES = { Side::N, Side::E, Side::S, Side::O };

/**
 * @brief Convierte un Side a su representación en texto.
 * inline evita costo de llamada a función y permite mantenerlo en el .h
 */
inline const char* sideToString(Side side) {
    switch (side) {
        case Side::N: return "N";
        case Side::E: return "E";
        case Side::S: return "S";
        case Side::O: return "O";
    }
    return "";
}

/**
 * @brief Retorna el lado opuesto para cálculos geométricos y de fronteras.
 */
inline Side getOppositeSide(Side side) {
    switch (side) {
        case Side::N: return Side::S;
        case Side::E: return Side::O;
        case Side::S: return Side::N;
        case Side::O: return Side::E;
    }
    return Side::N;
}

} // namespace domain

#endif // DOMAIN_SIDE_H