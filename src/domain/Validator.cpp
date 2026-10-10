#include "domain/Validator.h"
#include <sstream>

namespace domain {

// ============================================================================
// validate(vector) — sobrecarga de conveniencia
// ============================================================================

/**
 * @brief Valida una solución en formato vector.
 *
 * Verificación temprana: si alguna pieza está sin asignar (cellId < 0),
 * la solución se considera incompleta y se retorna {false, 0.0} sin
 * construir el mapa intermedio. Esto ahorra trabajo en el caso común
 * de soluciones parciales de las heurísticas.
 */
std::pair<bool, double> Validator::validate(const std::vector<int>& solution,
                                            const Snapshot& snapshot) {
    const std::vector<Piece>& pieces = snapshot.getPieces();
    if (solution.size() != pieces.size()) {
        return {false, 0.0};
    }

    // Verificación temprana: toda pieza debe estar asignada.
    for (std::size_t i = 0; i < solution.size(); ++i) {
        if (solution[i] < 0) {
            return {false, 0.0};
        }
    }

    // Convertir a mapa pieceId -> cellId.
    std::unordered_map<int, int> solutionById;
    solutionById.reserve(solution.size());
    for (std::size_t i = 0; i < solution.size(); ++i) {
        solutionById[pieces[i].getId()] = solution[i];
    }
    return validate(solutionById, snapshot);
}

// ============================================================================
// checkStructuralCompatibility — G_i == O_j (ecuación 3d)
// ============================================================================

/**
 * @brief Verifica si la pieza es estructuralmente compatible con la celda.
 *
 * Condición: cada lado gris de la pieza debe estar en un lado exterior
 * de la celda, y cada lado exterior debe estar en un lado gris de la pieza.
 * Es decir, los conjuntos G_i y O_j deben ser iguales.
 *
 * Costo: O(1) (a lo sumo 4 lados, 2 exteriores).
 */
bool Validator::checkStructuralCompatibility(const Piece& piece,
                                             const domain::OuterSides& outerSides) {
    int grayCount = 0;
    for (Side side : ALL_SIDES) {
        if (piece.getColor(side) == 0) {
            ++grayCount;
            if (!outerSides.contains(side)) {
                return false;  // Lado gris en posición interna
            }
        }
    }
    return grayCount == outerSides.count;  // Todos los exteriores deben ser grises
}

// ============================================================================
// validate(map) — validación completa (Listing 1)
// ============================================================================

std::pair<bool, double> Validator::validate(
    const std::unordered_map<int, int>& solution,
    const Snapshot& snapshot) {

    const auto& pieces = snapshot.getPieces();
    const auto& geometry = snapshot.getGeometry();
    const int numCells = geometry.getNumCells();

    // 1. Verificación de dimensión: todas las piezas deben estar.
    if (solution.size() != pieces.size()) {
        return {false, 0.0};
    }

    // Estructura de acceso directo O(1) para mapear celda -> pieza.
    std::vector<int> cellToPiece(numCells, -1);
    double totalCost = 0.0;

    // --- PASO 1: Validaciones individuales ---
    // Asignación única, capacidad, estructura, pistas y cálculo de costo.
    for (const auto& pair : solution) {
        const int pieceId = pair.first;
        const int cellId = pair.second;

        if (cellId < 0 || cellId >= numCells) {
            return {false, 0.0};
        }

        const Piece* piece = snapshot.findPiece(pieceId);
        if (piece == nullptr) {
            return {false, 0.0};
        }

        // Evitar superposición.
        if (cellToPiece[cellId] != -1) {
            return {false, 0.0};
        }

        // Capacidad K_j^t (0 si sellada).
        if (snapshot.getCellCapacity(cellId) == 0) {
            return {false, 0.0};
        }

        // Compatibilidad estructural (3d).
        OuterSides outerSides = geometry.getOuterSides(cellId);
        if (!checkStructuralCompatibility(*piece, outerSides)) {
            return {false, 0.0};
        }

        // Pista vigente (3e).
        const std::pair<Side, int>* clue = snapshot.getCellClue(cellId);
        if (clue != nullptr) {
            if (piece->getColor(clue->first) != clue->second) {
                return {false, 0.0};
            }
        }

        cellToPiece[cellId] = pieceId;
        totalCost += snapshot.getMoveCost(pieceId, cellId);
    }

    // --- PASO 2: Validar encaje entre celdas adyacentes (3f)-(3g) ---
    for (int cellId = 0; cellId < numCells; ++cellId) {
        const int pieceId = cellToPiece[cellId];
        if (pieceId == -1) continue;

        const Piece* thisPiece = snapshot.findPiece(pieceId);
        AdjacentCells adjacents = geometry.getAdjacentCells(cellId);

        for (int i = 0; i < adjacents.count; ++i) {
            const int neighborCellId = adjacents.cells[i].first;
            const Side sideFromThisCell = adjacents.cells[i].second;

            // Validar cada arista una sola vez.
            if (neighborCellId <= cellId) continue;

            const int neighborPieceId = cellToPiece[neighborCellId];
            if (neighborPieceId == -1) continue;

            const Piece* neighborPiece = snapshot.findPiece(neighborPieceId);
            const Side sideFromNeighbor = getOppositeSide(sideFromThisCell);

            if (thisPiece->getColor(sideFromThisCell) !=
                neighborPiece->getColor(sideFromNeighbor)) {
                return {false, 0.0};
            }
        }
    }

    return {true, totalCost};
}

// ============================================================================
// diagnoseInvalidity
// ============================================================================

std::string Validator::diagnoseInvalidity(
    const std::unordered_map<int, int>& solution,
    const Snapshot& snapshot) {

    std::ostringstream oss;
    const auto& pieces = snapshot.getPieces();
    const auto& geometry = snapshot.getGeometry();
    const int numCells = geometry.getNumCells();

    if (solution.size() != pieces.size()) {
        oss << "  * Tamaño incorrecto: se esperaban " << pieces.size()
            << " piezas, se recibieron " << solution.size() << ".\n";
        return oss.str();
    }

    std::vector<int> cellToPiece(numCells, -1);

    // Paso 1: verificar asignaciones individuales.
    for (const auto& pair : solution) {
        const int pieceId = pair.first;
        const int cellId  = pair.second;

        if (cellId < 0 || cellId >= numCells) {
            oss << "  * Pieza " << pieceId << " apunta a celda fuera de rango: "
                << cellId << ".\n";
            continue;
        }

        const Piece* piece = snapshot.findPiece(pieceId);
        if (piece == nullptr) {
            oss << "  * Pieza " << pieceId << " no existe.\n";
            continue;
        }

        if (snapshot.getCellCapacity(cellId) == 0) {
            oss << "  * Pieza " << pieceId
                << " intenta ocupar la celda sellada " << cellId << ".\n";
            continue;
        }

        if (cellToPiece[cellId] != -1) {
            oss << "  * Celda " << cellId << " duplicada: piezas "
                << cellToPiece[cellId] << " y " << pieceId << ".\n";
            continue;
        }

        if (!checkStructuralCompatibility(*piece,
                                          geometry.getOuterSides(cellId))) {
            oss << "  * Pieza " << pieceId
                << " no es estructuralmente compatible con la celda "
                << cellId << ".\n";
            continue;
        }

        const auto* clue = snapshot.getCellClue(cellId);
        if (clue != nullptr && piece->getColor(clue->first) != clue->second) {
            oss << "  * Pieza " << pieceId
                << " viola la pista de la celda " << cellId << ".\n";
            continue;
        }

        cellToPiece[cellId] = pieceId;
    }

    // Paso 2: verificar encaje entre vecinas.
    for (int cellId = 0; cellId < numCells; ++cellId) {
        if (cellToPiece[cellId] == -1) continue;

        const Piece* thisPiece = snapshot.findPiece(cellToPiece[cellId]);
        AdjacentCells adjacents = geometry.getAdjacentCells(cellId);

        for (int i = 0; i < adjacents.count; ++i) {
            const int neighborCellId = adjacents.cells[i].first;
            if (neighborCellId <= cellId) continue;

            const int neighborPieceId = cellToPiece[neighborCellId];
            if (neighborPieceId == -1) continue;

            const Piece* neighborPiece = snapshot.findPiece(neighborPieceId);
            const Side sideFromThis = adjacents.cells[i].second;
            const Side sideFromNeighbor = getOppositeSide(sideFromThis);

            if (thisPiece->getColor(sideFromThis) !=
                neighborPiece->getColor(sideFromNeighbor)) {
                oss << "  * Desajuste de color entre pieza "
                    << thisPiece->getId() << " (celda " << cellId
                    << ") y pieza " << neighborPiece->getId()
                    << " (celda " << neighborCellId << ").\n";
            }
        }
    }

    return oss.str();
}

} // namespace domain