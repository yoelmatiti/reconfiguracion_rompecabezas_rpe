#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unordered_map>
#include <sys/stat.h>
#include <sys/types.h>

#include "RpeParser.h"
#include "domain/Grid.h"
#include "domain/Validator.h"
#include "optimization/BranchAndBoundA0.h"
#include "optimization/CbcReferenceSolver.h"
#include "optimization/ClpRelaxationSolver.h"
#include "optimization/CSVExporter.h"
#include "optimization/HeuristicaH1.h"
#include "optimization/HeuristicaH2.h"
#include "optimization/Simulator.h"

namespace {

using namespace domain;
using namespace optimization;
using namespace parser;

// ============================================================================
// Estado de la sesión
// ============================================================================

struct RuntimeState {
    std::string currentCase;
    std::vector<std::string> caseHistory;
    std::vector<CSVRow> csvRows;
    parser::ParsedData parsed;
    bool hasLoadedCase = false;
};

// ============================================================================
// Helpers de UI
// ============================================================================

void printMenu() {
    std::cout << "\n=== RPE - Menú principal ===\n";
    std::cout << "1. Cargar caso .rpe\n";
    std::cout << "2. Mostrar y validar estado inicial\n";
    std::cout << "3. Ejecutar una época con A0, H1, H2 y R (comparativa)\n";
    std::cout << "4. Ejecutar la secuencia completa (multi-solver)\n";
    std::cout << "5. Exportar resultados CSV\n";
    std::cout << "6. Evaluación multi-semilla de GRASP (Sección 9)\n";
    std::cout << "7. Salir\n";
    std::cout << "Seleccione una opción: ";
}

std::string readLine() {
    std::string input;
    std::getline(std::cin, input);
    return input;
}

std::string trim(const std::string& value) {
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

bool ensureFileExists(const std::string& path) {
    std::ifstream test(path.c_str());
    return test.good();
}

std::string resolveCasePath(const std::string& rawPath) {
    std::string trimmed = trim(rawPath);
    if (trimmed.empty()) return "";

    if (ensureFileExists(trimmed)) return trimmed;

    std::string fallback = "instances/" + trimmed;
    if (ensureFileExists(fallback)) return fallback;

    return "";
}

std::string formatPlacement(const std::unordered_map<int, int>& placement) {
    return CSVExporter::placementToString(placement);
}

// ============================================================================
// Recarga automática del caso
// ============================================================================

bool reloadCurrentCase(RuntimeState& state) {
    if (state.currentCase.empty()) return false;

    try {
        parser::ParsedData parsed = RpeParser::parse(state.currentCase);
        state.parsed = std::move(parsed);
        state.hasLoadedCase = true;
        std::cout << "[auto-recarga] Caso restaurado desde: "
                  << state.currentCase << "\n";
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[auto-recarga] Error: " << e.what() << "\n";
        state.hasLoadedCase = false;
        return false;
    }
}

bool ensureCaseReady(RuntimeState& state) {
    if (state.hasLoadedCase) return true;

    if (!state.currentCase.empty()) {
        std::cout << "El caso fue consumido. Recargando automáticamente...\n";
        return reloadCurrentCase(state);
    }

    std::cout << "Primero debe cargar un caso .rpe.\n";
    return false;
}

// ============================================================================
// Opción 1: Cargar caso .rpe
// ============================================================================

void loadCase(RuntimeState& state) {
    std::cout << "Ingrese la ruta del archivo .rpe: ";
    std::string input = readLine();
    std::string path = resolveCasePath(input);

    if (path.empty()) {
        std::cout << "Ruta vacía o archivo no encontrado. "
                     "Se buscó en la ruta ingresada y en instances/.\n";
        return;
    }

    try {
        parser::ParsedData parsed = RpeParser::parse(path);
        state.parsed = std::move(parsed);
        state.currentCase = path;
        state.hasLoadedCase = true;
        state.csvRows.clear();
        state.caseHistory.push_back(path);

        std::cout << "Caso cargado correctamente desde: " << path << "\n";
    } catch (const std::exception& e) {
        std::cout << "Error al cargar el caso: " << e.what() << "\n";
        state.hasLoadedCase = false;
    }
}

// ============================================================================
// Opción 2: Mostrar y validar estado inicial
// ============================================================================

void printSnapshotSummary(RuntimeState& state) {
    if (!ensureCaseReady(state)) return;

    const auto& grid = *state.parsed.grid;
    const auto& pieces = grid.getPieces();
    const auto& cells = grid.getCells();

    auto snapshot = state.parsed.grid->createSnapshot();
    if (!snapshot) {
        std::cout << "No se pudo crear el snapshot del caso cargado.\n";
        return;
    }

    std::cout << "\n=== Estado inicial del caso: " << state.currentCase
              << " ===\n";
    std::cout << "Piezas: " << pieces.size() << "\n";
    std::cout << "Celdas: " << cells.size() << "\n";
    std::cout << "Eventos programados: "
              << state.parsed.epochCommands.size() << "\n";
    std::cout << "Semilla: " << state.parsed.seed << "\n";
    std::cout << "time_ms: " << state.parsed.timeLimitMs << "\n";
    std::cout << "grasp_eta: " << state.parsed.graspEta << "\n";
    std::cout << "grasp_lambda: " << state.parsed.graspLambda << "\n";

    std::unordered_map<int, int> placement;
    for (size_t i = 0; i < pieces.size(); ++i) {
        placement[pieces[i].getId()] = pieces[i].getInitialCell();
    }

    auto validation = domain::Validator::validate(placement, *snapshot);
    std::cout << "Validación inicial: "
              << (validation.first ? "VÁLIDA" : "INVÁLIDA") << "\n";
    std::cout << "Costo objetivo inicial: " << validation.second << "\n\n";

    std::cout << "Resumen de celdas:\n";
    for (size_t i = 0; i < cells.size(); ++i) {
        const auto& cell = cells[i];
        std::cout << "  Celda " << i << ": ";
        if (cell.isSealed()) {
            std::cout << "SELLADA";
        } else if (cell.hasClue()) {
            auto clue = cell.getClue();
            std::cout << "CON_PISTA (" << static_cast<int>(clue->first)
                      << ", " << clue->second << ")";
        } else {
            std::cout << "ABIERTA";
        }
        std::cout << "\n";
    }
}

// ============================================================================
// Opción 3: Ejecutar UNA época con los 4 métodos (comparativa)
// ============================================================================

void runEpochWithMethods(RuntimeState& state) {
    if (!ensureCaseReady(state)) return;

    const auto startTime = std::chrono::steady_clock::now();
    std::cout << "\n=== Ejecutando una época con A0, H1, H2 y R ===\n";

    auto snapshot = state.parsed.grid->createSnapshot();
    if (!snapshot) {
        std::cout << "No se pudo crear el snapshot inicial.\n";
        return;
    }

    const int timeLimitMs = state.parsed.timeLimitMs > 0
                            ? state.parsed.timeLimitMs : 5000;
    const double timeLimitSeconds = timeLimitMs / 1000.0;

    // --- A0 ---
    {
        const auto t0 = std::chrono::steady_clock::now();
        A0Config cfg;
        cfg.maxTimeSeconds = timeLimitSeconds;
        BranchAndBoundA0 a0(*snapshot, cfg);
        auto a0Result = a0.solve();
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        std::cout << "A0: " << statusToString(a0Result.status)
                  << " | objetivo: " << a0Result.upperBound
                  << " | tiempo: " << elapsed << " s\n";

        CSVRow row;
        row.epoch = 0; row.timestamp = 0; row.method = "A0";
        row.status = statusToString(a0Result.status);
        row.objective = a0Result.upperBound;
        row.moved_pieces = static_cast<int>(a0Result.incumbent.size());
        row.runtime_ms = elapsed * 1000.0;
        row.LB = a0Result.lowerBound;
        row.UB = a0Result.upperBound;
        row.gap_percent = a0Result.gap;
        row.nodes = a0Result.nodesExplored;
        row.lp_calls = a0Result.lpCalls;
        row.placement = formatPlacement(a0Result.incumbent);
        state.csvRows.push_back(row);
    }

    // --- H1 ---
    {
        const auto t0 = std::chrono::steady_clock::now();
        Solution h1Sol = HeuristicH1::solve(*snapshot, Solution(),
                                             timeLimitSeconds);
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        std::unordered_map<int, int> h1Map;
        for (size_t i = 0; i < h1Sol.size(); ++i) {
            if (h1Sol[i] >= 0) h1Map[snapshot->getPieces()[i].getId()] = h1Sol[i];
        }
        auto h1Val = domain::Validator::validate(h1Map, *snapshot);

        std::cout << "H1: " << (h1Val.first ? "OK" : "INVALID")
                  << " | objetivo: " << h1Val.second
                  << " | tiempo: " << elapsed << " s\n";

        CSVRow row;
        row.epoch = 0; row.timestamp = 0; row.method = "H1";
        row.status = h1Val.first ? "OK" : "INVALID";
        row.objective = h1Val.second;
        row.moved_pieces = static_cast<int>(h1Map.size());
        row.runtime_ms = elapsed * 1000.0;
        row.LB = 0.0; row.UB = h1Val.second; row.gap_percent = 0.0;
        row.nodes = 0; row.lp_calls = 0;
        row.placement = formatPlacement(h1Map);
        state.csvRows.push_back(row);
    }

    // --- H2 (GRASP) ---
    {
        const auto t0 = std::chrono::steady_clock::now();

        HeuristicH2::Params h2Params;
        h2Params.eta = state.parsed.graspEta;
        h2Params.lambda = state.parsed.graspLambda;
        h2Params.baseSeed = state.parsed.seed;
        h2Params.epoch = 0;
        h2Params.maxTimeSeconds = timeLimitSeconds;
        h2Params.maxIterations = 100;

        Solution h2Sol = HeuristicH2::solve(*snapshot, h2Params);
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        std::unordered_map<int, int> h2Map;
        for (size_t i = 0; i < h2Sol.size(); ++i) {
            if (h2Sol[i] >= 0) h2Map[snapshot->getPieces()[i].getId()] = h2Sol[i];
        }
        auto h2Val = domain::Validator::validate(h2Map, *snapshot);

        std::cout << "H2: " << (h2Val.first ? "OK" : "INVALID")
                  << " | objetivo: " << h2Val.second
                  << " | tiempo: " << elapsed << " s\n";

        CSVRow row;
        row.epoch = 0; row.timestamp = 0; row.method = "H2";
        row.status = h2Val.first ? "OK" : "INVALID";
        row.objective = h2Val.second;
        row.moved_pieces = static_cast<int>(h2Map.size());
        row.runtime_ms = elapsed * 1000.0;
        row.LB = 0.0; row.UB = h2Val.second; row.gap_percent = 0.0;
        row.nodes = 0; row.lp_calls = 0;
        row.placement = formatPlacement(h2Map);
        state.csvRows.push_back(row);
    }

    // --- R ---
    {
        const auto t0 = std::chrono::steady_clock::now();
        SolverConfig cfg;
        cfg.maxTimeSeconds = timeLimitSeconds;
        cfg.logLevel = 0;
        cfg.useCuts = true;
        cfg.useHeuristics = true;
        auto cbcResult = CbcReferenceSolver::solve(*snapshot, cfg);
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        std::cout << "R:  " << statusToString(cbcResult.status)
                  << " | objetivo: " << cbcResult.objectiveValue
                  << " | tiempo: " << elapsed << " s\n";

        CSVRow row;
        row.epoch = 0; row.timestamp = 0; row.method = "R";
        row.status = statusToString(cbcResult.status);
        row.objective = cbcResult.objectiveValue;
        row.moved_pieces = static_cast<int>(cbcResult.solution.size());
        row.runtime_ms = elapsed * 1000.0;
        row.LB = cbcResult.bestLowerBound;
        row.UB = cbcResult.objectiveValue;
        row.gap_percent = 0.0;
        row.nodes = cbcResult.nodeCount;
        row.lp_calls = 0;
        row.placement = formatPlacement(cbcResult.solution);
        state.csvRows.push_back(row);
    }

    const auto totalElapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - startTime).count();
    std::cout << "\nResumen comparativo ejecutado en "
              << totalElapsed << " s.\n";
}

// ============================================================================
// Opción 4: Ejecutar la secuencia completa (multi-solver por época)
// ============================================================================

void runFullSequence(RuntimeState& state) {
    if (!ensureCaseReady(state)) return;

    std::cout << "\n=== Ejecutando secuencia completa (multi-solver) ===\n";

    auto grid = std::move(state.parsed.grid);
    auto epochCommands = std::move(state.parsed.epochCommands);

    if (!grid) {
        std::cout << "Error: no hay grid válido.\n";
        state.hasLoadedCase = false;
        return;
    }

    Simulator simulator(std::move(*grid),
                        std::move(epochCommands),
                        state.parsed.graspEta,
                        state.parsed.graspLambda,
                        SolverType::H1,
                        true);

    simulator.setBaseSeed(state.parsed.seed);
    simulator.setMaxIterations(100);

    auto results = simulator.run();

    for (const auto& epochResult : results) {
        std::unordered_map<int, int> placement;
        for (size_t i = 0; i < epochResult.solution.size(); ++i) {
            if (epochResult.solution[i] >= 0) {
                placement[static_cast<int>(i)] = epochResult.solution[i];
            }
        }

        CSVRow row;
        row.epoch = epochResult.epochIndex;
        row.timestamp = epochResult.timestamp;
        row.method = epochResult.solverName.empty()
                     ? "SIMULATOR" : epochResult.solverName;
        row.status = epochResult.isValid ? "OK" : "INVALID";
        row.objective = epochResult.objectiveValue;
        row.moved_pieces = static_cast<int>(placement.size());
        row.runtime_ms = epochResult.executionTimeSeconds * 1000.0;
        row.LB = 0.0;
        row.UB = epochResult.objectiveValue;
        row.gap_percent = 0.0;
        row.nodes = 0;
        row.lp_calls = 0;
        row.placement = formatPlacement(placement);
        state.csvRows.push_back(row);
    }

    std::cout << "Secuencia completa finalizada. "
              << results.size() << " épocas registradas.\n";
    std::cout << "El caso se recargará automáticamente si es necesario.\n";

    state.hasLoadedCase = false;  // Marcado para auto-recarga
}

// ============================================================================
// Opción 5: Exportar resultados a CSV
// ============================================================================

void exportResults(RuntimeState& state) {
    if (state.csvRows.empty()) {
        std::cout << "No hay resultados para exportar. "
                     "Ejecute primero las opciones 3 o 4.\n";
        return;
    }

    mkdir("results", 0777);

    const std::string path = "results/out_results.csv";
    if (CSVExporter::writeResults(path, state.csvRows)) {
        std::cout << "Resultados exportados en: " << path << "\n";
    } else {
        std::cout << "No se pudo exportar el CSV.\n";
    }
}

// ============================================================================
// Opción 6: Evaluación multi-semilla de GRASP (Sección 9)
// ============================================================================

void runGraspMultiSeed(RuntimeState& state) {
    if (!ensureCaseReady(state)) return;

    std::cout << "\n=== Evaluación Multi-Semilla de GRASP (Sección 9) ===\n";

    // --- Parámetros interactivos ---
    std::cout << "Número de semillas [default 5]: ";
    std::string seedInput = trim(readLine());
    int numSeeds = 5;
    if (!seedInput.empty()) {
        try { numSeeds = std::stoi(seedInput); }
        catch (...) { numSeeds = 5; }
    }
    if (numSeeds < 1) numSeeds = 1;
    if (numSeeds > 100) numSeeds = 100;

    std::cout << "Tiempo por semilla en ms [default = time_ms del caso]: ";
    std::string timeInput = trim(readLine());
    int timeLimitMs = state.parsed.timeLimitMs > 0
                      ? state.parsed.timeLimitMs : 5000;
    if (!timeInput.empty()) {
        try { timeLimitMs = std::stoi(timeInput); }
        catch (...) { /* usar default */ }
    }
    if (timeLimitMs < 100) timeLimitMs = 100;

    // --- Ejecutar evaluación ---
    // Creamos un Simulator temporal con una copia profunda del grid actual.
    // El grid original del estado NO se modifica.
    Simulator simulator(state.parsed.grid->deepCopy(),
                        domain::EpochCommands(),
                        state.parsed.graspEta,
                        state.parsed.graspLambda,
                        SolverType::H2,
                        false);
    simulator.setBaseSeed(state.parsed.seed);
    simulator.setMaxIterations(100);

    std::cout << "\nEjecutando GRASP con " << numSeeds
              << " semillas (" << timeLimitMs << " ms c/u)...\n";

    GraspMultiSeedResult multiRes = simulator.runGraspMultiSeed(
        numSeeds, timeLimitMs, 0);

    // --- Registro en CSV dedicado ---
    std::vector<GraspSeedRow> graspRows;
    graspRows.reserve(multiRes.perSeed.size());

    for (const auto& seedRes : multiRes.perSeed) {
        GraspSeedRow row;
        row.seed = seedRes.seed;
        row.epoch = seedRes.epoch;
        row.objective = seedRes.objective;
        row.runtimeMs = seedRes.runtimeMs;
        row.feasibleConstructions = seedRes.feasibleConstructions;
        row.bestFound = (seedRes.seed == multiRes.bestSeed) &&
                        seedRes.bestFound;
        graspRows.push_back(row);
    }

    mkdir("results", 0777);
    const std::string graspPath = "results/grasp_multiseed.csv";
    if (CSVExporter::writeGraspMultiSeed(graspPath, graspRows)) {
        std::cout << "Resultados multi-semilla exportados en: "
                  << graspPath << "\n";
    } else {
        std::cout << "No se pudo exportar el CSV multi-semilla.\n";
    }

    // --- Resumen estadístico en consola ---
    std::cout << "\n=== Resumen estadístico ===\n";
    std::cout << "Semillas evaluadas: " << multiRes.numSeeds << "\n";
    std::cout << "Semillas factibles: " << multiRes.numFeasible << "\n";

    if (multiRes.numFeasible > 0) {
        std::cout << "Mejor costo (min Z):    " << multiRes.bestCost
                  << "  (semilla " << multiRes.bestSeed << ")\n";
        std::cout << "Peor costo (max Z):     " << multiRes.worstCost << "\n";
        std::cout << "Costo promedio (Z̄):     " << multiRes.avgCost << "\n";
        std::cout << "Desv. estándar (σ):     " << multiRes.stdDevCost << "\n";
        std::cout << "Tiempo promedio:        " << multiRes.avgRuntimeMs
                  << " ms\n";

        // Registrar el mejor resultado en el CSV global (como si fuera
        // una fila del método H2 con la mejor semilla).
        std::unordered_map<int, int> bestPlacement;
        for (size_t i = 0; i < multiRes.bestSolution.size(); ++i) {
            if (multiRes.bestSolution[i] >= 0) {
                bestPlacement[static_cast<int>(i)] = multiRes.bestSolution[i];
            }
        }

        CSVRow bestRow;
        bestRow.epoch = 0;
        bestRow.timestamp = 0;
        bestRow.method = "H2";
        bestRow.status = "OK";
        bestRow.objective = multiRes.bestCost;
        bestRow.moved_pieces = static_cast<int>(bestPlacement.size());
        bestRow.runtime_ms = multiRes.avgRuntimeMs;
        bestRow.LB = 0.0;
        bestRow.UB = multiRes.bestCost;
        bestRow.gap_percent = 0.0;
        bestRow.nodes = 0;
        bestRow.lp_calls = 0;
        bestRow.placement = formatPlacement(bestPlacement);
        state.csvRows.push_back(bestRow);
    } else {
        std::cout << "Ninguna semilla encontró solución factible.\n";
    }
}

} // namespace

// ============================================================================
// main
// ============================================================================
// ============================================================================
// main (Soporta modo CLI y menú interactivo)
// ============================================================================

int main(int argc, char* argv[]) {
    // ------------------------------------------------------------------------
    // Modo Batch / CLI (usado por el script de Python)
    // ------------------------------------------------------------------------
    if (argc > 1) {
        std::string rawPath = "";
        double eta = 0.5;
        double lambda = 0.0;
        uint64_t seed = 42;
        int timeLimitMs = 2000;

        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--instance" && i + 1 < argc) rawPath = argv[++i];
            else if (arg == "--eta" && i + 1 < argc) eta = std::atof(argv[++i]);
            else if (arg == "--lambda" && i + 1 < argc) lambda = std::atof(argv[++i]);
            else if (arg == "--seed" && i + 1 < argc) seed = std::strtoull(argv[++i], nullptr, 10);
            else if (arg == "--time_ms" && i + 1 < argc) timeLimitMs = std::atoi(argv[++i]);
        }

        std::string resolvedPath = resolveCasePath(rawPath);
        if (resolvedPath.empty()) {
            std::cerr << "Error: No se encontró la instancia " << rawPath << "\n";
            return 1;
        }

        try {
            auto parsed = parser::RpeParser::parse(resolvedPath);
            Simulator simulator(parsed.grid->deepCopy(),
                                domain::EpochCommands(),
                                eta, lambda,
                                SolverType::H2, false);
            simulator.setBaseSeed(seed);
            simulator.setMaxIterations(100);

            // Ejecuta GRASP para la semilla específica
            GraspMultiSeedResult multiRes = simulator.runGraspMultiSeed(1, timeLimitMs, 0);

            if (multiRes.numFeasible > 0) {
                std::cout << "RESULT," << resolvedPath << "," << eta << "," << lambda << ","
                          << seed << "," << multiRes.bestCost << "," << multiRes.avgRuntimeMs << "\n";
            } else {
                std::cout << "RESULT," << resolvedPath << "," << eta << "," << lambda << ","
                          << seed << ",INFEASIBLE," << timeLimitMs << "\n";
            }
        } catch (const std::exception& e) {
            std::cerr << "Error en ejecución CLI: " << e.what() << "\n";
            return 1;
        }
        return 0;
    }

    // ------------------------------------------------------------------------
    // Modo Interactivo (Menú por defecto)
    // ------------------------------------------------------------------------
    RuntimeState state;

    while (true) {
        printMenu();
        std::string input = trim(readLine());

        if (input.empty()) {
            std::cout << "Entrada vacía. Intente nuevamente.\n";
            continue;
        }

        if (input == "1") {
            loadCase(state);
        } else if (input == "2") {
            printSnapshotSummary(state);
        } else if (input == "3") {
            runEpochWithMethods(state);
        } else if (input == "4") {
            runFullSequence(state);
        } else if (input == "5") {
            exportResults(state);
        } else if (input == "6") {
            runGraspMultiSeed(state);
        } else if (input == "7") {
            std::cout << "Saliendo del programa.\n";
            break;
        } else {
            std::cout << "Opción no válida. Intente nuevamente.\n";
        }
    }

    return 0;
}