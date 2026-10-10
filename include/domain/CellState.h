#ifndef DOMAIN_CELLSTATE_H
#define DOMAIN_CELLSTATE_H

#include <memory>
#include <stdexcept>
#include <utility>
#include "domain/Side.h"

namespace domain {

/**
 * @brief Interfaz base para el estado de una celda (Patrón State).
 *
 * Cada estado concreto (Open, Clue, Sealed) implementa las transiciones
 * permitidas según la tabla normativa de la sección 4.2 del enunciado.
 *
 * Reglas de transición:
 *   - OpenState   : acepta SELLAR y PISTA. LIBERAR es inválido.
 *   - ClueState   : acepta SELLAR, PISTA (sobrescribe) y LIBERAR.
 *   - SealedState : acepta solo LIBERAR. SELLAR y PISTA son inválidos.
 *
 * Las transiciones inválidas lanzan std::runtime_error con un mensaje
 * descriptivo que incluye el nombre del estado de origen.
 *
 * Todas las transiciones retornan un nuevo estado (o nullptr si no hay
 * cambio), delegando en Cell la responsabilidad de actualizar su puntero.
 */
class CellState {
public:
    virtual ~CellState() = default;

    /**
     * @brief Ejecuta la transición SELLAR.
     *
     * @param clueSide Salida: lado de la pista vigente (si aplica).
     * @param clueColor Salida: color de la pista vigente (si aplica).
     * @return unique_ptr<CellState> Nuevo estado, o nullptr si no cambia.
     * @throws std::runtime_error si la transición es inválida.
     */
    virtual std::unique_ptr<CellState> handleSellar() const = 0;

    /**
     * @brief Ejecuta la transición PISTA.
     *
     * @param side Lado de la pista (debe ser no exterior).
     * @param color Color de la pista.
     * @return unique_ptr<CellState> Nuevo estado, o nullptr si no cambia.
     * @throws std::runtime_error si la transición es inválida.
     */
    virtual std::unique_ptr<CellState> handlePista(Side side, int color) const = 0;

    /**
     * @brief Ejecuta la transición LIBERAR.
     *
     * @return unique_ptr<CellState> Nuevo estado, o nullptr si no cambia.
     * @throws std::runtime_error si la transición es inválida.
     */
    virtual std::unique_ptr<CellState> handleLiberar() const = 0;

    /**
     * @brief Clona el estado actual (para copia de Cell).
     */
    virtual std::unique_ptr<CellState> clone() const = 0;

    // --- Consultas de tipo (evitan dynamic_cast en bucles calientes) ---
    virtual bool isOpen()   const noexcept = 0;
    virtual bool isClue()   const noexcept = 0;
    virtual bool isSealed() const noexcept = 0;

    // --- Consulta de pista vigente (solo válido en ClueState) ---
    virtual Side getClueSide()  const noexcept { return Side::N; }
    virtual int  getClueColor() const noexcept { return 0; }

    // --- Nombre del estado (para mensajes de error) ---
    virtual const char* name() const noexcept = 0;
};

// ============================================================================
// Estados concretos
// ============================================================================

/**
 * @brief Estado ABIERTA: la celda puede recibir una pieza.
 *
 * Transiciones válidas:
 *   SELLAR -> SealedState
 *   PISTA  -> ClueState
 *   LIBERAR es inválido (la celda ya está abierta).
 */
class OpenState : public CellState {
public:
    std::unique_ptr<CellState> handleSellar() const override;
    std::unique_ptr<CellState> handlePista(Side side, int color) const override;
    std::unique_ptr<CellState> handleLiberar() const override;
    std::unique_ptr<CellState> clone() const override;

    bool isOpen()   const noexcept override { return true; }
    bool isClue()   const noexcept override { return false; }
    bool isSealed() const noexcept override { return false; }
    const char* name() const noexcept override { return "ABIERTA"; }
};

/**
 * @brief Estado CON_PISTA: la celda está abierta pero exige un color en un lado.
 *
 * Transiciones válidas:
 *   SELLAR -> SealedState (la pista se pierde)
 *   PISTA  -> ClueState (nueva pista, sobrescribe)
 *   LIBERAR -> OpenState (elimina la pista)
 */
class ClueState : public CellState {
public:
    ClueState(Side side, int color);

    std::unique_ptr<CellState> handleSellar() const override;
    std::unique_ptr<CellState> handlePista(Side side, int color) const override;
    std::unique_ptr<CellState> handleLiberar() const override;
    std::unique_ptr<CellState> clone() const override;

    bool isOpen()   const noexcept override { return true; }
    bool isClue()   const noexcept override { return true; }
    bool isSealed() const noexcept override { return false; }
    const char* name() const noexcept override { return "CON_PISTA"; }

    Side getClueSide()  const noexcept override { return clueSide_; }
    int  getClueColor() const noexcept override { return clueColor_; }

private:
    Side clueSide_;
    int  clueColor_;
};

/**
 * @brief Estado SELLADA: la celda no admite pieza.
 *
 * Transiciones válidas:
 *   LIBERAR -> OpenState
 *   SELLAR y PISTA son inválidos.
 */
class SealedState : public CellState {
public:
    std::unique_ptr<CellState> handleSellar() const override;
    std::unique_ptr<CellState> handlePista(Side side, int color) const override;
    std::unique_ptr<CellState> handleLiberar() const override;
    std::unique_ptr<CellState> clone() const override;

    bool isOpen()   const noexcept override { return false; }
    bool isClue()   const noexcept override { return false; }
    bool isSealed() const noexcept override { return true; }
    const char* name() const noexcept override { return "SELLADA"; }
};

} // namespace domain

#endif // DOMAIN_CELLSTATE_H