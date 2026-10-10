#ifndef DOMAIN_VALIDATOR_H
#define DOMAIN_VALIDATOR_H

#include <utility>
#include <vector>
#include <unordered_map>
#include <string>
#include "domain/Snapshot.h"
#include "domain/Side.h"
#include "domain/GridGeometry.h"

namespace domain {

/**
 * @brief Clase utilitaria para validar una solución contra un Snapshot.
 *
 * Implementa las verificaciones del Listing 1 del enunciado y el cálculo
 * de Z^t(x).
 */
class Validator {
public:
    /**
     * @brief Valida una solución en formato vector (índice = pieceIdx).
     *
     * @param solution Vector donde solution[i] = cellId de la pieza i.
     * @param snapshot Snapshot de la época t.
     * @return Par (esValida, costoTotal).
     */
    static std::pair<bool, double> validate(const std::vector<int>& solution,
                                            const Snapshot& snapshot);

    /**
     * @brief Valida una solución en formato mapa (pieceId -> cellId).
     */
    static std::pair<bool, double> validate(
        const std::unordered_map<int, int>& solution,
        const Snapshot& snapshot);

    /**
     * @brief Retorna un mensaje descriptivo del porqué de la invalidación.
     *
     * @return String con todos los errores encontrados, o vacío si es válida.
     */
    static std::string diagnoseInvalidity(
        const std::unordered_map<int, int>& solution,
        const Snapshot& snapshot);

    /**
     * @brief Verifica si los lados grises de la pieza coinciden exactamente
     *        con los bordes exteriores de la celda (G_i == O_j).
     */
    static bool checkStructuralCompatibility(const Piece& piece,
                                              const OuterSides& outerSides);
};

} // namespace domain

#endif // DOMAIN_VALIDATOR_H