#ifndef DOMAIN_CELL_H
#define DOMAIN_CELL_H

#include <memory>
#include <utility>
#include "domain/Side.h"
#include "domain/CellState.h"

namespace domain {

/**
 * @brief Entidad que representa una Celda del tablero.
 *
 * Gestiona su estado a través del Patrón State. Las transiciones válidas
 * están centralizadas en las clases derivadas de CellState. Cualquier
 * transición inválida lanza std::runtime_error.
 *
 * La celda también almacena la pista vigente (lado + color) cuando su
 * estado es CON_PISTA.
 */
class Cell {
public:
    /**
     * @brief Constructor.
     *
     * Inicializa la celda en estado ABIERTA sin pista.
     *
     * @param id Identificador único de la celda (>= 0).
     * @throws std::invalid_argument si id < 0.
     */
    explicit Cell(int id);

    /**
     * @brief Constructor de copia (copia profunda del estado).
     */
    Cell(const Cell& other);

    /**
     * @brief Operador de asignación (copia profunda del estado).
     */
    Cell& operator=(const Cell& other);

    /**
     * @brief Constructor de movimiento.
     */
    Cell(Cell&& other) noexcept;

    /**
     * @brief Operador de asignación por movimiento.
     */
    Cell& operator=(Cell&& other) noexcept;

    // --- Consultas ---
    int getId() const noexcept { return id_; }

    bool isOpen()   const noexcept { return state_->isOpen(); }
    bool isClue()   const noexcept { return state_->isClue(); }
    bool isSealed() const noexcept { return state_->isSealed(); }

    // Alias para compatibilidad con código existente.
    bool hasClue()  const noexcept { return state_->isClue(); }

    /**
     * @brief Capacidad K_j^t de la celda.
     *
     * @return 1 si ABIERTA o CON_PISTA, 0 si SELLADA.
     */
    int getCapacity() const noexcept {
        return state_->isSealed() ? 0 : 1;
    }

    /**
     * @brief Retorna la pista vigente o nullptr si no hay pista.
     *
     * El puntero apunta a un par (Side, int) almacenado en la celda,
     * válido mientras la celda no cambie de estado.
     */
    const std::pair<Side, int>* getClue() const noexcept;

    // --- Transiciones de estado ---
    /**
     * @brief Aplica SELLAR.
     * @throws std::runtime_error si la transición es inválida.
     */
    void applySellar();

    /**
     * @brief Aplica PISTA.
     * @throws std::runtime_error si la transición es inválida.
     * @throws std::invalid_argument si color < 0.
     */
    void applyPista(Side clueSide, int color);

    /**
     * @brief Aplica LIBERAR.
     * @throws std::runtime_error si la transición es inválida.
     */
    void applyLiberar();

private:
    int id_;
    std::unique_ptr<CellState> state_;
    // Pista vigente (solo válida si state_->isClue()).
    std::pair<Side, int> clue_;
};

} // namespace domain

#endif // DOMAIN_CELL_H