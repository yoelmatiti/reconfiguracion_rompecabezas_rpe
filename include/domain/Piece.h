#ifndef DOMAIN_PIECE_H
#define DOMAIN_PIECE_H

#include <array>
#include <cstddef>
#include "domain/Side.h"

namespace domain {

/**
 * @brief Entidad que representa una Pieza del rompecabezas.
 *
 * Almacena su identificador, los colores en sus 4 lados (sin rotación),
 * el costo de manipulación y la celda inicial.
 *
 * Convención de índices: colors_[0]=N, colors_[1]=E, colors_[2]=S, colors_[3]=O.
 */
class Piece {
public:
    /**
     * @brief Constructor.
     *
     * @param id Identificador único (>= 0).
     * @param north Color del lado Norte.
     * @param east Color del lado Este.
     * @param south Color del lado Sur.
     * @param west Color del lado Oeste.
     * @param cost Costo de manipulación (> 0).
     * @param initialCell Celda inicial en la época 0 (>= 0).
     * @throws std::invalid_argument si cost <= 0 o initialCell < 0.
     */
    Piece(int id, int north, int east, int south, int west,
          double cost, int initialCell);

    // --- Getters ---
    int getId() const noexcept { return id_; }
    int getColor(Side side) const;
    const std::array<int, 4>& getColors() const noexcept { return colors_; }
    double getCost() const noexcept { return cost_; }
    int getInitialCell() const noexcept { return initialCell_; }

    /**
     * @brief Cuenta los lados con color 0 (gris / borde exterior).
     *
     * Complejidad: O(1).
     *
     * @return Número de lados grises en [0, 4].
     */
    int countGraySides() const noexcept;

    /**
     * @brief Verifica la validez del patrón de lados grises.
     *
     * Condiciones (sección 6.5 del enunciado):
     *  - |G_i| <= 2.
     *  - Si |G_i| == 2, los dos lados deben ser adyacentes.
     *
     * Pares adyacentes válidos: (N,E), (E,S), (S,O), (O,N).
     * Pares opuestos inválidos: (N,S), (E,O).
     *
     * Complejidad: O(1).
     *
     * @return true si el patrón es válido.
     */
    bool hasValidGrayPattern() const noexcept;

private:
    int id_;
    std::array<int, 4> colors_;  // Orden: N, E, S, O
    double cost_;
    int initialCell_;
};

} // namespace domain

#endif // DOMAIN_PIECE_H