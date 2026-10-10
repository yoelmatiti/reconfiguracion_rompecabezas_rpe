#ifndef PARSER_RPEPARSER_H
#define PARSER_RPEPARSER_H

#include <string>
#include <memory>
#include <vector>
#include <map>
#include <fstream>

#include "domain/Grid.h"
#include "domain/Command.h"     // ← Provee CellCommand, SellarCommand, etc.
#include "domain/Side.h"
#include "domain/GridGeometry.h"

namespace parser {

/**
 * @brief Estructura DTO que contiene toda la información parseada del archivo .rpe.
 */
struct ParsedData {
    std::string name;
    int timeLimitMs = 0;
    unsigned long long seed = 0;
    double graspEta = 0.0;
    double graspLambda = 0.0;

    std::unique_ptr<domain::Grid> grid;
    domain::EpochCommands epochCommands;   // = std::map<int, CommandBatch>
                                           //   CommandBatch = std::vector<std::unique_ptr<CellCommand>>
};

/**
 * @brief Parser para archivos .rpe versión 1.
 */
class RpeParser {
public:
    static ParsedData parse(const std::string& filename);

private:
    explicit RpeParser(const std::string& filename);

    ParsedData parseAll();

    void parseVersion(const std::vector<std::string>& tokens);
    void parseName(const std::vector<std::string>& tokens);
    void parseControl(const std::vector<std::string>& tokens);

    void parseBoardBlock();
    void parseCellsBlock();
    void parsePiecesBlock();
    void parseEventsBlock();

    bool getNextLineTokens(std::vector<std::string>& tokens, std::string& rawLine);
    domain::Side parseSide(const std::string& sideStr);
    [[noreturn]] void throwError(const std::string& message) const;

    std::ifstream file_;
    int lineNumber_;
    ParsedData data_;

    std::unique_ptr<domain::GridGeometry> geometry_;
    std::vector<domain::Cell> cells_;
    std::vector<domain::Piece> pieces_;
};

} // namespace parser

#endif // PARSER_RPEPARSER_H