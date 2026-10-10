#include "RpeParser.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace parser {

// ============================================================================
// Constantes del formato y helpers locales
// ============================================================================

namespace {

    // --- Palabras clave del formato .rpe ---
    const std::string RPE_VERSION  = "RPE_VERSION";
    const std::string VERSION_VALUE= "1";
    const std::string NAME         = "NAME";
    const std::string CONTROL      = "CONTROL";
    const std::string BOARD_BEGIN  = "BOARD_BEGIN";
    const std::string BOARD_END    = "BOARD_END";
    const std::string CELLS_BEGIN  = "CELLS_BEGIN";
    const std::string CELLS_END    = "CELLS_END";
    const std::string PIECES_BEGIN = "PIECES_BEGIN";
    const std::string PIECES_END   = "PIECES_END";
    const std::string EVENTS_BEGIN = "EVENTS_BEGIN";
    const std::string EVENTS_END   = "EVENTS_END";

    const std::string ROWS         = "ROWS";
    const std::string COLS         = "COLS";
    const std::string COLORS       = "COLORS";
    const std::string SELLAR       = "SELLAR";
    const std::string PISTA        = "PISTA";
    const std::string LIBERAR      = "LIBERAR";

    // --- Helpers de conversión con validación estricta ---

    int parseNonNegativeInt(const std::string& token) {
        if (token.empty()) {
            throw std::invalid_argument("Token vacío donde se esperaba entero.");
        }
        if (token[0] == '-') {
            throw std::invalid_argument("Valor negativo no permitido: " + token);
        }
        try {
            size_t pos = 0;
            const int value = std::stoi(token, &pos);
            if (pos != token.size()) {
                throw std::invalid_argument("Entero mal formado: " + token);
            }
            return value;
        } catch (const std::out_of_range&) {
            throw std::invalid_argument("Entero fuera de rango: " + token);
        }
    }

    int parsePositiveInt(const std::string& token) {
        const int value = parseNonNegativeInt(token);
        if (value <= 0) {
            throw std::invalid_argument("Se requiere entero positivo: " + token);
        }
        return value;
    }

    double parseFiniteDouble(const std::string& token) {
        if (token.empty()) {
            throw std::invalid_argument("Token vacío donde se esperaba double.");
        }
        try {
            size_t pos = 0;
            const double value = std::stod(token, &pos);
            if (pos != token.size()) {
                throw std::invalid_argument("Double mal formado: " + token);
            }
            if (!std::isfinite(value)) {
                throw std::invalid_argument("Valor no finito: " + token);
            }
            return value;
        } catch (const std::out_of_range&) {
            throw std::invalid_argument("Double fuera de rango: " + token);
        }
    }

    uint64_t parseUint64(const std::string& token) {
        if (token.empty()) {
            throw std::invalid_argument("Token vacío donde se esperaba uint64.");
        }
        if (token[0] == '-') {
            throw std::invalid_argument("Semilla no puede ser negativa: " + token);
        }
        try {
            size_t pos = 0;
            const uint64_t value = std::stoull(token, &pos);
            if (pos != token.size()) {
                throw std::invalid_argument("uint64 mal formado: " + token);
            }
            return value;
        } catch (const std::out_of_range&) {
            throw std::invalid_argument("uint64 fuera de rango: " + token);
        }
    }

} // namespace

// ============================================================================
// Punto de entrada público
// ============================================================================

ParsedData RpeParser::parse(const std::string& filename) {
    RpeParser parser(filename);
    return parser.parseAll();
}

// ============================================================================
// Constructor
// ============================================================================

RpeParser::RpeParser(const std::string& filename)
    : file_(filename), lineNumber_(0) {
    if (!file_.is_open()) {
        throw std::runtime_error("No se pudo abrir el archivo: " + filename);
    }
}

// ============================================================================
// Orquestación principal
// ============================================================================

ParsedData RpeParser::parseAll() {
    std::vector<std::string> tokens;
    std::string rawLine;

    while (getNextLineTokens(tokens, rawLine)) {
        if (tokens[0] == RPE_VERSION) {
            parseVersion(tokens);
        } else if (tokens[0] == NAME) {
            parseName(tokens);
        } else if (tokens[0] == CONTROL) {
            parseControl(tokens);
        } else if (tokens[0] == BOARD_BEGIN) {
            parseBoardBlock();
        } else if (tokens[0] == CELLS_BEGIN) {
            parseCellsBlock();
        } else if (tokens[0] == PIECES_BEGIN) {
            parsePiecesBlock();
        } else if (tokens[0] == EVENTS_BEGIN) {
            parseEventsBlock();
        } else {
            throwError("Token desconocido fuera de bloque: " + tokens[0]);
        }
    }

    if (!geometry_) {
        throwError("El archivo no contenía un bloque BOARD_BEGIN válido.");
    }

    data_.grid.reset(new domain::Grid(
        std::move(geometry_),
        std::move(pieces_),
        std::move(cells_)
    ));

    return std::move(data_);
}

// ============================================================================
// Lectura línea a línea
// ============================================================================

bool RpeParser::getNextLineTokens(std::vector<std::string>& tokens,
                                  std::string& rawLine) {
    tokens.clear();

    while (std::getline(file_, rawLine)) {
        ++lineNumber_;

        const size_t commentPos = rawLine.find('#');
        if (commentPos != std::string::npos) {
            rawLine = rawLine.substr(0, commentPos);
        }

        std::istringstream iss(rawLine);
        std::string token;
        while (iss >> token) {
            tokens.push_back(token);
        }

        if (!tokens.empty()) return true;
    }
    return false;
}

// ============================================================================
// Error helper
// ============================================================================

void RpeParser::throwError(const std::string& message) const {
    throw std::runtime_error(
        "Error en línea " + std::to_string(lineNumber_) + ": " + message);
}

// ============================================================================
// RPE_VERSION / NAME / CONTROL
// ============================================================================

void RpeParser::parseVersion(const std::vector<std::string>& tokens) {
    if (tokens.size() != 2 || tokens[1] != VERSION_VALUE) {
        throwError("Versión RPE inválida. Se esperaba: "
                   + RPE_VERSION + " " + VERSION_VALUE);
    }
}

void RpeParser::parseName(const std::vector<std::string>& tokens) {
    if (tokens.size() != 2) {
        throwError("Formato NAME inválido. Se esperaba: NAME <nombre>");
    }
    if (tokens[1].empty()) {
        throwError("NAME no puede estar vacío.");
    }
    data_.name = tokens[1];
}

void RpeParser::parseControl(const std::vector<std::string>& tokens) {
    if (tokens.size() != 5) {
        throwError("Formato CONTROL inválido. Se esperaba: "
                   "CONTROL <time_ms> <seed> <grasp_eta> <grasp_lambda>");
    }

    try {
        data_.timeLimitMs = parsePositiveInt(tokens[1]);
        data_.seed        = parseUint64(tokens[2]);
        data_.graspEta    = parseFiniteDouble(tokens[3]);
        data_.graspLambda = parseFiniteDouble(tokens[4]);
    } catch (const std::exception& e) {
        throwError(std::string("CONTROL inválido: ") + e.what());
    }

    if (data_.graspEta < 0.0 || data_.graspEta > 1.0) {
        throwError("grasp_eta debe estar en [0, 1].");
    }
    if (data_.graspLambda < 0.0) {
        throwError("grasp_lambda debe ser no negativo.");
    }
}

// ============================================================================
// Bloque BOARD
// ============================================================================

void RpeParser::parseBoardBlock() {
    int rows = 0, cols = 0, colors = 0;
    std::vector<std::string> tokens;
    std::string rawLine;

    while (getNextLineTokens(tokens, rawLine)) {
        if (tokens[0] == BOARD_END) {
            if (rows <= 0)   throwError("ROWS debe ser positivo.");
            if (cols <= 0)   throwError("COLS debe ser positivo.");
            if (colors < 2)  throwError("COLORS debe ser >= 2 (color 0 reservado).");

            geometry_.reset(new domain::GridGeometry(rows, cols, colors));
            return;
        }

        try {
            if (tokens[0] == ROWS && tokens.size() == 2) {
                rows = parsePositiveInt(tokens[1]);
            } else if (tokens[0] == COLS && tokens.size() == 2) {
                cols = parsePositiveInt(tokens[1]);
            } else if (tokens[0] == COLORS && tokens.size() == 2) {
                colors = parsePositiveInt(tokens[1]);
            } else {
                throwError("Token inesperado en BOARD: " + tokens[0]);
            }
        } catch (const std::runtime_error&) {
            throw;
        } catch (const std::exception& e) {
            throwError(std::string("Valor inválido en BOARD: ") + e.what());
        }
    }
    throwError("Falta BOARD_END.");
}

// ============================================================================
// Bloque CELLS
// ============================================================================

void RpeParser::parseCellsBlock() {
    std::vector<std::string> tokens;
    std::string rawLine;

    const int expectedCells = geometry_ ? geometry_->getNumCells() : -1;
    const int numColors     = geometry_ ? geometry_->getNumColors() : -1;
    int expectedId = 0;

    while (getNextLineTokens(tokens, rawLine)) {
        if (tokens[0] == CELLS_END) {
            if (expectedCells > 0 &&
                static_cast<int>(cells_.size()) != expectedCells) {
                throwError("CELLS debe contener exactamente "
                           + std::to_string(expectedCells)
                           + " celdas (R·C), se encontraron "
                           + std::to_string(cells_.size()) + ".");
            }
            return;
        }

        if (tokens.size() != 4) {
            throwError("Formato de celda inválido. Se esperaba: "
                       "<cell_id> <state> <clue_side> <clue_color>");
        }

        try {
            const int cellId = parseNonNegativeInt(tokens[0]);
            if (cellId != expectedId) {
                throwError("cell_id fuera de secuencia: se esperaba "
                           + std::to_string(expectedId) + ", se obtuvo "
                           + std::to_string(cellId) + ".");
            }
            ++expectedId;

            domain::Cell cell(cellId);

            if (tokens[1] == "SELLADA") {
                if (tokens[2] != "-" || tokens[3] != "-") {
                    throwError("Celda SELLADA no debe especificar pista.");
                }
                cell.applySellar();
            }
            else if (tokens[1] == "CON_PISTA") {
                if (tokens[2] == "-" || tokens[3] == "-") {
                    throwError("Celda CON_PISTA debe especificar lado y color.");
                }

                const domain::Side side = parseSide(tokens[2]);
                const int color = parseNonNegativeInt(tokens[3]);

                if (numColors > 0 && (color < 1 || color >= numColors)) {
                    throwError("Color de pista fuera de rango [1, K-1].");
                }

                if (geometry_) {
                    const domain::OuterSides outer =
                        geometry_->getOuterSides(cellId);
                    if (outer.contains(side)) {
                        throwError("Pista sobre lado exterior de la celda.");
                    }
                }

                cell.applyPista(side, color);
            }
            else if (tokens[1] == "ABIERTA") {
                if (tokens[2] != "-" || tokens[3] != "-") {
                    throwError("Celda ABIERTA no debe especificar pista.");
                }
            }
            else {
                throwError("Estado de celda desconocido: " + tokens[1]);
            }

            cells_.push_back(std::move(cell));
        }
        catch (const std::runtime_error&) {
            throw;
        }
        catch (const std::exception& e) {
            throwError(std::string("Valores de celda inválidos: ") + e.what());
        }
    }
    throwError("Falta CELLS_END.");
}

// ============================================================================
// Bloque PIECES
// ============================================================================

void RpeParser::parsePiecesBlock() {
    std::vector<std::string> tokens;
    std::string rawLine;

    const int expectedCells = geometry_ ? geometry_->getNumCells() : -1;
    const int numColors     = geometry_ ? geometry_->getNumColors() : -1;
    int expectedId = 0;

    while (getNextLineTokens(tokens, rawLine)) {
        if (tokens[0] == PIECES_END) {
            if (pieces_.empty()) {
                throwError("PIECES no puede estar vacío.");
            }
            return;
        }

        if (tokens.size() != 7) {
            throwError("Formato de pieza inválido. Se esperaba: "
                       "<id> <N> <E> <S> <O> <cost> <initial_cell>");
        }

        try {
            const int id      = parseNonNegativeInt(tokens[0]);
            const int north   = parseNonNegativeInt(tokens[1]);
            const int east    = parseNonNegativeInt(tokens[2]);
            const int south   = parseNonNegativeInt(tokens[3]);
            const int west    = parseNonNegativeInt(tokens[4]);
            const double cost = parseFiniteDouble(tokens[5]);
            const int cell    = parseNonNegativeInt(tokens[6]);

            if (id != expectedId) {
                throwError("piece_id fuera de secuencia: se esperaba "
                           + std::to_string(expectedId) + ", se obtuvo "
                           + std::to_string(id) + ".");
            }
            ++expectedId;

            if (cost <= 0.0) {
                throwError("El costo de la pieza debe ser positivo.");
            }

            if (numColors > 0) {
                if (north < 0 || north >= numColors ||
                    east  < 0 || east  >= numColors ||
                    south < 0 || south >= numColors ||
                    west  < 0 || west  >= numColors) {
                    throwError("Color de pieza fuera de rango [0, K-1].");
                }
            }

            if (expectedCells > 0 && (cell < 0 || cell >= expectedCells)) {
                throwError("celda_inicial fuera de rango [0, R·C-1].");
            }

            domain::Piece piece(id, north, east, south, west, cost, cell);

            // Validación clave: patrón de lados grises.
            if (!piece.hasValidGrayPattern()) {
                throwError(
                    "Pieza " + std::to_string(id) +
                    ": patrón de lados grises inválido. Se permite a lo sumo "
                    "2 lados con color 0 y, si son 2, deben ser adyacentes.");
            }

            pieces_.push_back(std::move(piece));
        }
        catch (const std::runtime_error&) {
            throw;
        }
        catch (const std::exception& e) {
            throwError(std::string("Valores de pieza inválidos: ") + e.what());
        }
    }
    throwError("Falta PIECES_END.");
}

// ============================================================================
// Bloque EVENTS
// ============================================================================
//
// CAMBIO CLAVE: los comandos se construyen como std::unique_ptr<CellCommand>
// (clase base del Patrón Command), no como std::unique_ptr<Command>.
// Esto es el único cambio respecto a la versión anterior.
// ============================================================================

void RpeParser::parseEventsBlock() {
    std::vector<std::string> tokens;
    std::string rawLine;

    int currentTimestamp = -1;
    domain::CommandBatch currentBatch;    // = vector<unique_ptr<CellCommand>>
    std::set<int> cellsInCurrentBatch;

    const int expectedCells = geometry_ ? geometry_->getNumCells() : -1;
    const int numColors     = geometry_ ? geometry_->getNumColors() : -1;

    while (getNextLineTokens(tokens, rawLine)) {
        if (tokens[0] == EVENTS_END) {
            if (currentTimestamp != -1 && !currentBatch.empty()) {
                data_.epochCommands[currentTimestamp] = std::move(currentBatch);
            }
            return;
        }

        if (tokens.size() < 3) {
            throwError("Evento mal formado.");
        }

        try {
            const int timestamp = parseNonNegativeInt(tokens[0]);
            const std::string& eventType = tokens[1];

            if (currentTimestamp != -1 && timestamp < currentTimestamp) {
                throwError("Timestamps fuera de orden: "
                           + std::to_string(timestamp) + " < "
                           + std::to_string(currentTimestamp) + ".");
            }

            if (timestamp != currentTimestamp && currentTimestamp != -1) {
                data_.epochCommands[currentTimestamp] = std::move(currentBatch);
                currentBatch.clear();
                cellsInCurrentBatch.clear();
            }
            currentTimestamp = timestamp;

            // --- SELLAR ---
            if (eventType == SELLAR) {
                if (tokens.size() != 3) {
                    throwError("SELLAR requiere 1 argumento: <cell_id>.");
                }
                const int cellId = parseNonNegativeInt(tokens[2]);

                if (expectedCells > 0 && (cellId < 0 || cellId >= expectedCells)) {
                    throwError("SELLAR: celda fuera de rango.");
                }
                if (cellsInCurrentBatch.count(cellId) > 0) {
                    throwError("SELLAR: celda duplicada en el mismo timestamp.");
                }
                cellsInCurrentBatch.insert(cellId);

                // *** FIX: CellCommand en lugar de Command ***
                currentBatch.push_back(
                    std::unique_ptr<domain::CellCommand>(
                        new domain::SellarCommand(cellId)));
            }
            // --- PISTA ---
            else if (eventType == PISTA) {
                if (tokens.size() != 5) {
                    throwError("PISTA requiere 3 argumentos: "
                               "<cell_id> <side> <color>.");
                }
                const int cellId = parseNonNegativeInt(tokens[2]);
                const domain::Side side = parseSide(tokens[3]);
                const int color = parseNonNegativeInt(tokens[4]);

                if (expectedCells > 0 && (cellId < 0 || cellId >= expectedCells)) {
                    throwError("PISTA: celda fuera de rango.");
                }
                if (numColors > 0 && (color < 1 || color >= numColors)) {
                    throwError("PISTA: color fuera de rango [1, K-1].");
                }
                if (geometry_) {
                    const domain::OuterSides outer =
                        geometry_->getOuterSides(cellId);
                    if (outer.contains(side)) {
                        throwError("PISTA: lado exterior no permitido.");
                    }
                }
                if (cellsInCurrentBatch.count(cellId) > 0) {
                    throwError("PISTA: celda duplicada en el mismo timestamp.");
                }
                cellsInCurrentBatch.insert(cellId);

                // *** FIX: CellCommand en lugar de Command ***
                currentBatch.push_back(
                    std::unique_ptr<domain::CellCommand>(
                        new domain::PistaCommand(cellId, side, color)));
            }
            // --- LIBERAR ---
            else if (eventType == LIBERAR) {
                if (tokens.size() != 3) {
                    throwError("LIBERAR requiere 1 argumento: <cell_id>.");
                }
                const int cellId = parseNonNegativeInt(tokens[2]);

                if (expectedCells > 0 && (cellId < 0 || cellId >= expectedCells)) {
                    throwError("LIBERAR: celda fuera de rango.");
                }
                if (cellsInCurrentBatch.count(cellId) > 0) {
                    throwError("LIBERAR: celda duplicada en el mismo timestamp.");
                }
                cellsInCurrentBatch.insert(cellId);

                // *** FIX: CellCommand en lugar de Command ***
                currentBatch.push_back(
                    std::unique_ptr<domain::CellCommand>(
                        new domain::LiberarCommand(cellId)));
            }
            else {
                throwError("Evento desconocido: " + eventType);
            }
        }
        catch (const std::runtime_error&) {
            throw;
        }
        catch (const std::exception& e) {
            throwError(std::string("Evento inválido: ") + e.what());
        }
    }
    throwError("Falta EVENTS_END.");
}

// ============================================================================
// Utilidades
// ============================================================================

domain::Side RpeParser::parseSide(const std::string& sideStr) {
    if (sideStr == "N") return domain::Side::N;
    if (sideStr == "E") return domain::Side::E;
    if (sideStr == "S") return domain::Side::S;
    if (sideStr == "O") return domain::Side::O;
    throwError("Lado inválido: " + sideStr);
}

} // namespace parser