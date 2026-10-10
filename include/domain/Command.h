#ifndef DOMAIN_COMMAND_H
#define DOMAIN_COMMAND_H

#include <memory>
#include <vector>
#include <map>
#include <string>

#include "domain/Grid.h"
#include "domain/Side.h"

namespace domain {

// Forward declaration
class Grid;

/**
 * @brief Interfaz base para el Patrón Command.
 *
 * Encapsula una acción que puede ejecutarse sobre el Grid. La firma de
 * `execute` retorna `bool` para indicar éxito/fracaso, permitiendo al
 * Simulator aplicar lotes atómicamente con rollback.
 *
 * Reglas de implementación:
 *  - `execute()` debe retornar `true` si la transición fue aplicada.
 *  - `execute()` debe retornar `false` si la transición es inválida.
 *  - `execute()` NO debe lanzar excepciones (capturarlas internamente).
 *  - `execute()` debe ser `const` para permitir ejecución desde referencias
 *    constantes y no modificar el estado interno del comando.
 */
class CellCommand {
public:
    virtual ~CellCommand() = default;

    /**
     * @brief Ejecuta el comando sobre el grid.
     *
     * @param grid Referencia al grid que será modificado.
     * @return true si el comando se aplicó correctamente.
     * @return false si la transición es inválida (sin modificar el grid).
     */
    virtual bool execute(Grid& grid) const = 0;

    /**
     * @brief Obtiene una descripción del comando para logging/debug.
     */
    virtual std::string toString() const = 0;
};

// ============================================================================
// Comandos concretos
// ============================================================================

/**
 * @brief Comando para sellar una celda.
 */
class SellarCommand : public CellCommand {
public:
    explicit SellarCommand(int cellId);
    bool execute(Grid& grid) const override;
    std::string toString() const override;

private:
    int cellId_;
};

/**
 * @brief Comando para aplicar una pista a una celda.
 */
class PistaCommand : public CellCommand {
public:
    PistaCommand(int cellId, Side side, int color);
    bool execute(Grid& grid) const override;
    std::string toString() const override;

private:
    int cellId_;
    Side side_;
    int color_;
};

/**
 * @brief Comando para liberar una celda.
 */
class LiberarCommand : public CellCommand {
public:
    explicit LiberarCommand(int cellId);
    bool execute(Grid& grid) const override;
    std::string toString() const override;

private:
    int cellId_;
};

// ============================================================================
// Alias para agrupar comandos
// ============================================================================

/**
 * @brief Lote de comandos que deben aplicarse atómicamente.
 */
using CommandBatch = std::vector<std::unique_ptr<CellCommand>>;

/**
 * @brief Mapa de época (timestamp) -> lote de comandos.
 */
using EpochCommands = std::map<int, CommandBatch>;

} // namespace domain

#endif // DOMAIN_COMMAND_H