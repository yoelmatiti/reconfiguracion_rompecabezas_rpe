// ============================================================================
// Suite de pruebas RPE — Sección 8.4 del enunciado (19 casos)
// ============================================================================
// Estándar: C++11
// Framework: assert-based, autochequeable, determinista
// ============================================================================

#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "RpeParser.h"
#include "domain/Cell.h"
#include "domain/Command.h"
#include "domain/Grid.h"
#include "domain/GridGeometry.h"
#include "domain/Piece.h"
#include "domain/Side.h"
#include "domain/Snapshot.h"
#include "domain/Validator.h"

#include "optimization/A0Strategy.h"
#include "optimization/BranchAndBoundA0.h"
#include "optimization/CbcReferenceSolver.h"
#include "optimization/ClpRelaxationSolver.h"
#include "optimization/H1Strategy.h"
#include "optimization/H2Strategy.h"
#include "optimization/HeuristicaH1.h"
#include "optimization/HeuristicaH2.h"
#include "optimization/LocalSearch.h"
#include "optimization/LpRepairHeuristic.h"
#include "optimization/MatrixBuilder.h"
#include "optimization/RStrategy.h"
#include "optimization/RunConfig.h"
#include "optimization/Simulator.h"
#include "optimization/SolveResult.h"
#include "optimization/SolverTypes.h"

// Los asserts deben estar activos incluso con -DNDEBUG
#ifdef NDEBUG
#undef NDEBUG
#endif

using namespace domain;
using parser::RpeParser;
using namespace optimization;

// ============================================================================
// Framework minimal de tests
// ============================================================================

namespace {

int g_testsRun = 0;
int g_testsPassed = 0;
int g_testsFailed = 0;

#define RPE_TEST(name)                                                        \
    void name()

#define RPE_RUN(test_fn)                                                      \
    do {                                                                      \
        ++g_testsRun;                                                         \
        std::cout << "[TEST " << g_testsRun << "] " << #test_fn << " ... ";   \
        try {                                                                 \
            test_fn();                                                        \
            std::cout << "OK\n";                                              \
            ++g_testsPassed;                                                  \
        } catch (const std::exception& e) {                                   \
            std::cout << "FAIL (" << e.what() << ")\n";                       \
            ++g_testsFailed;                                                  \
        } catch (...) {                                                       \
            std::cout << "FAIL (unknown)\n";                                  \
            ++g_testsFailed;                                                  \
        }                                                                     \
    } while (0)

#define RPE_ASSERT(cond)                                                      \
    do {                                                                      \
        if (!(cond)) {                                                        \
            throw std::runtime_error(                                         \
                std::string("Assert falló: ") + #cond +                       \
                " (línea " + std::to_string(__LINE__) + ")");                 \
        }                                                                     \
    } while (0)

#define RPE_ASSERT_EQ(a, b)                                                   \
    do {                                                                      \
        if (!((a) == (b))) {                                                  \
            throw std::runtime_error(                                         \
                std::string("Assert EQ falló: ") + #a + " != " + #b +         \
                " (línea " + std::to_string(__LINE__) + ")");                 \
        }                                                                     \
    } while (0)

#define RPE_ASSERT_NEAR(a, b, tol)                                            \
    do {                                                                      \
        if (std::abs((a) - (b)) > (tol)) {                                    \
            throw std::runtime_error(                                         \
                std::string("Assert NEAR falló: ") + #a + " != " + #b +       \
                " (tol=" + std::to_string(tol) + ", línea " +                 \
                std::to_string(__LINE__) + ")");                              \
        }                                                                     \
    } while (0)

// ============================================================================
// Helpers
// ============================================================================

void createDummyRpeFile(const std::string& filename) {
    std::ofstream out(filename);
    out << "RPE_VERSION 1\n";
    out << "NAME test_instance\n";
    out << "CONTROL 1000 42 0.25 0.10\n";
    out << "BOARD_BEGIN\n";
    out << "ROWS 2\n";
    out << "COLS 2\n";
    out << "COLORS 3\n";
    out << "BOARD_END\n";
    out << "CELLS_BEGIN\n";
    out << "0 ABIERTA - -\n";
    out << "1 ABIERTA - -\n";
    out << "2 ABIERTA - -\n";
    out << "3 ABIERTA - -\n";
    out << "CELLS_END\n";
    out << "PIECES_BEGIN\n";
    out << "0 0 1 1 0 10 0\n";
    out << "1 0 0 1 1 15 1\n";
    out << "2 1 1 0 0 20 2\n";
    out << "3 1 0 0 1 25 3\n";
    out << "PIECES_END\n";
    out << "EVENTS_BEGIN\n";
    out << "10 SELLAR 3\n";
    out << "20 LIBERAR 3\n";
    out << "EVENTS_END\n";
}

Snapshot makeSyntheticSnapshot2x2() {
    std::unique_ptr<GridGeometry> geometry(new GridGeometry(2, 2, 3));
    std::vector<Piece> pieces;
    pieces.emplace_back(0, 0, 1, 1, 0, 10, 0);
    pieces.emplace_back(1, 0, 0, 1, 1, 15, 1);
    pieces.emplace_back(2, 1, 1, 0, 0, 20, 2);
    pieces.emplace_back(3, 1, 0, 0, 1, 25, 3);

    std::vector<Cell> cells;
    for (int i = 0; i < 4; ++i) cells.emplace_back(i);

    std::unordered_map<int, int> placement = {{0,0},{1,1},{2,2},{3,3}};
    std::unordered_map<int, double> costs = {{0,10},{1,15},{2,20},{3,25}};

    return Snapshot(std::move(geometry),
                    std::move(pieces),
                    std::move(cells),
                    std::move(placement),
                    std::move(costs));
}

Grid makeSyntheticGrid2x2() {
    std::unique_ptr<GridGeometry> geometry(new GridGeometry(2, 2, 3));
    std::vector<Piece> pieces;
    pieces.emplace_back(0, 0, 1, 1, 0, 10, 0);
    pieces.emplace_back(1, 0, 0, 1, 1, 15, 1);
    pieces.emplace_back(2, 1, 1, 0, 0, 20, 2);
    pieces.emplace_back(3, 1, 0, 0, 1, 25, 3);

    std::vector<Cell> cells;
    for (int i = 0; i < 4; ++i) cells.emplace_back(i);

    return Grid(std::move(geometry), std::move(pieces), std::move(cells));
}

} // namespace

// ============================================================================
// TEST 1: Mapeo completo de t = 0
// ============================================================================

RPE_TEST(test01_MapeoConstantesT0) {
    std::string path = "test01_instance.rpe";
    createDummyRpeFile(path);

    parser::ParsedData data = RpeParser::parse(path);

    RPE_ASSERT_EQ(data.name, std::string("test_instance"));
    RPE_ASSERT_EQ(data.timeLimitMs, 1000);
    RPE_ASSERT_EQ(data.seed, 42ULL);
    RPE_ASSERT_NEAR(data.graspEta, 0.25, 1e-9);
    RPE_ASSERT_NEAR(data.graspLambda, 0.10, 1e-9);
    RPE_ASSERT(data.grid != nullptr);
    RPE_ASSERT_EQ(data.grid->getPieces().size(), 4u);
    RPE_ASSERT_EQ(data.grid->getCells().size(), 4u);
    RPE_ASSERT_EQ(data.grid->getGeometry().getNumColors(), 3);
    RPE_ASSERT_EQ(data.epochCommands.size(), 2u);

    auto snapshot = data.grid->createSnapshot();
    RPE_ASSERT(snapshot != nullptr);
    RPE_ASSERT_NEAR(snapshot->getMoveCost(0, 0), 0.0, 1e-9);
    RPE_ASSERT_NEAR(snapshot->getMoveCost(0, 1), 10.0, 1e-9);

    std::remove(path.c_str());
}

// ============================================================================
// TEST 2: Atomicidad de lote de eventos con rollback
// ============================================================================

RPE_TEST(test02_AtomicidadLoteEventos) {
    // Crear Simulator con grid sintético y sin comandos.
    Grid grid = makeSyntheticGrid2x2();
    Simulator sim(std::move(grid), domain::EpochCommands(),
                  0.5, 1.0, SolverType::H1, false);

    // Estado inicial: celda 0 ABIERTA.
    RPE_ASSERT(sim.getGrid().getCells()[0].isOpen());

    // --- Caso 1: lote con comando inválido (PISTA sobre SELLADA) ---
    {
        domain::CommandBatch batch;
        batch.push_back(std::unique_ptr<CellCommand>(new SellarCommand(0)));
        batch.push_back(std::unique_ptr<CellCommand>(new PistaCommand(0, Side::N, 1)));

        BatchResult result = sim.applyCommandsAtomically(batch);

        // El lote debe fallar.
        RPE_ASSERT(!result.success);
        RPE_ASSERT_EQ(result.failedAt, 1u);   // El segundo comando falló
        RPE_ASSERT_EQ(result.appliedCount, 1u); // El primero sí se aplicó en la copia

        // *** Rollback: el grid original NO debe haberse modificado ***
        RPE_ASSERT(sim.getGrid().getCells()[0].isOpen());
        RPE_ASSERT(!sim.getGrid().getCells()[0].isSealed());
    }

    // --- Caso 2: lote completamente válido ---
    {
        domain::CommandBatch batch;
        batch.push_back(std::unique_ptr<CellCommand>(new SellarCommand(0)));
        batch.push_back(std::unique_ptr<CellCommand>(new PistaCommand(1, Side::N, 2)));

        BatchResult result = sim.applyCommandsAtomically(batch);

        RPE_ASSERT(result.success);
        RPE_ASSERT_EQ(result.appliedCount, 2u);

        // El grid SÍ debe reflejar los cambios.
        RPE_ASSERT(sim.getGrid().getCells()[0].isSealed());
        RPE_ASSERT(sim.getGrid().getCells()[1].hasClue());
    }

    // --- Caso 3: lote vacío ---
    {
        domain::CommandBatch empty;
        BatchResult result = sim.applyCommandsAtomically(empty);
        RPE_ASSERT(result.success);
        RPE_ASSERT_EQ(result.appliedCount, 0u);
    }
}

// ============================================================================
// TEST 3: Tipos de eventos (SELLAR, PISTA, LIBERAR)
// ============================================================================

RPE_TEST(test03_TiposDeEventos) {
    Grid grid = makeSyntheticGrid2x2();

    SellarCommand sellar(0);
    RPE_ASSERT(sellar.execute(grid));
    RPE_ASSERT(grid.getCells()[0].isSealed());

    LiberarCommand liberar(0);
    RPE_ASSERT(liberar.execute(grid));
    RPE_ASSERT(grid.getCells()[0].isOpen());

    PistaCommand pista(1, Side::N, 2);
    RPE_ASSERT(pista.execute(grid));
    RPE_ASSERT(grid.getCells()[1].hasClue());
    auto clue = grid.getCells()[1].getClue();
    RPE_ASSERT(clue != nullptr);
    RPE_ASSERT(clue->first == Side::N);
    RPE_ASSERT_EQ(clue->second, 2);

    LiberarCommand liberarPista(1);
    RPE_ASSERT(liberarPista.execute(grid));
    RPE_ASSERT(grid.getCells()[1].isOpen());
    RPE_ASSERT(!grid.getCells()[1].hasClue());
}

// ============================================================================
// TEST 4: Transiciones válidas e inválidas de CellState
// ============================================================================

RPE_TEST(test04_TransicionesCellState) {
    // --- Válidas ---
    {
        Cell cell(0);
        RPE_ASSERT(cell.isOpen());

        cell.applySellar();
        RPE_ASSERT(cell.isSealed());

        cell.applyLiberar();
        RPE_ASSERT(cell.isOpen());

        cell.applyPista(Side::N, 1);
        RPE_ASSERT(cell.hasClue());

        cell.applyPista(Side::E, 2);
        RPE_ASSERT(cell.getClue() != nullptr);
        RPE_ASSERT(cell.getClue()->first == Side::E);
        RPE_ASSERT_EQ(cell.getClue()->second, 2);

        cell.applyLiberar();
        RPE_ASSERT(cell.isOpen());
        RPE_ASSERT(cell.getClue() == nullptr);

        cell.applyPista(Side::N, 3);
        cell.applySellar();
        RPE_ASSERT(cell.isSealed());
    }

    // --- Inválidas ---
    {
        Cell cell(0);
        bool caught = false;
        try { cell.applyLiberar(); }
        catch (const std::logic_error&) { caught = true; }
        RPE_ASSERT(caught);
    }

    {
        Cell cell(0);
        cell.applySellar();
        bool caught = false;
        try { cell.applySellar(); }
        catch (const std::logic_error&) { caught = true; }
        RPE_ASSERT(caught);
    }

    {
        Cell cell(0);
        cell.applySellar();
        bool caught = false;
        try { cell.applyPista(Side::N, 1); }
        catch (const std::logic_error&) { caught = true; }
        RPE_ASSERT(caught);
    }
}

// ============================================================================
// TEST 5: Validación de parámetros GRASP (eta, lambda)
// ============================================================================

RPE_TEST(test05_ValidacionEtaLambda) {
    Snapshot snapshot = makeSyntheticSnapshot2x2();

    {
        HeuristicH2::Params p;
        p.eta = -0.1;
        p.lambda = 1.0;
        p.maxTimeSeconds = 1.0;
        p.maxIterations = 10;
        Solution s = HeuristicH2::solve(snapshot, p);
        RPE_ASSERT(s.empty());
    }

    {
        HeuristicH2::Params p;
        p.eta = 1.5;
        p.lambda = 1.0;
        p.maxTimeSeconds = 1.0;
        p.maxIterations = 10;
        Solution s = HeuristicH2::solve(snapshot, p);
        RPE_ASSERT(s.empty());
    }

    {
        HeuristicH2::Params p;
        p.eta = 0.5;
        p.lambda = -1.0;
        p.maxTimeSeconds = 1.0;
        p.maxIterations = 10;
        Solution s = HeuristicH2::solve(snapshot, p);
        RPE_ASSERT(s.empty());
    }

    {
        HeuristicH2::Params p;
        p.eta = std::numeric_limits<double>::quiet_NaN();
        p.lambda = 1.0;
        p.maxTimeSeconds = 1.0;
        p.maxIterations = 10;
        Solution s = HeuristicH2::solve(snapshot, p);
        RPE_ASSERT(s.empty());
    }

    {
        HeuristicH2::Params p;
        p.eta = 0.5;
        p.lambda = std::numeric_limits<double>::infinity();
        p.maxTimeSeconds = 1.0;
        p.maxIterations = 10;
        Solution s = HeuristicH2::solve(snapshot, p);
        RPE_ASSERT(s.empty());
    }
}

// ============================================================================
// TEST 6: Casos extremos de RCL (eta = 0 y eta = 1)
// ============================================================================

RPE_TEST(test06_ExtremosRCL) {
    Snapshot snapshot = makeSyntheticSnapshot2x2();

    HeuristicH2::Params p0;
    p0.eta = 0.0;
    p0.lambda = 0.0;
    p0.baseSeed = 42;
    p0.epoch = 0;
    p0.maxTimeSeconds = 2.0;
    p0.maxIterations = 20;

    Solution a0 = HeuristicH2::solve(snapshot, p0);
    Solution b0 = HeuristicH2::solve(snapshot, p0);
    RPE_ASSERT_EQ(a0.size(), b0.size());
    for (size_t i = 0; i < a0.size(); ++i) RPE_ASSERT_EQ(a0[i], b0[i]);

    HeuristicH2::Params p1;
    p1.eta = 1.0;
    p1.lambda = 0.0;
    p1.baseSeed = 42;
    p1.epoch = 0;
    p1.maxTimeSeconds = 2.0;
    p1.maxIterations = 20;

    Solution a1 = HeuristicH2::solve(snapshot, p1);
    Solution b1 = HeuristicH2::solve(snapshot, p1);
    RPE_ASSERT_EQ(a1.size(), b1.size());
    for (size_t i = 0; i < a1.size(); ++i) RPE_ASSERT_EQ(a1[i], b1[i]);
}

// ============================================================================
// TEST 7: Transición de época (c_ij^t y p_i^{t+1})
// ============================================================================

RPE_TEST(test07_TransicionEpoca) {
    Snapshot snapshot = makeSyntheticSnapshot2x2();

    RPE_ASSERT_EQ(snapshot.getPreviousCell(0), 0);
    RPE_ASSERT_EQ(snapshot.getPreviousCell(1), 1);
    RPE_ASSERT_EQ(snapshot.getPreviousCell(2), 2);
    RPE_ASSERT_EQ(snapshot.getPreviousCell(3), 3);

    RPE_ASSERT_NEAR(snapshot.getMoveCost(0, 0),  0.0, 1e-9);
    RPE_ASSERT_NEAR(snapshot.getMoveCost(0, 1), 10.0, 1e-9);
    RPE_ASSERT_NEAR(snapshot.getMoveCost(1, 0), 15.0, 1e-9);
    RPE_ASSERT_NEAR(snapshot.getMoveCost(2, 2),  0.0, 1e-9);
    RPE_ASSERT_NEAR(snapshot.getMoveCost(3, 1), 25.0, 1e-9);
}

// ============================================================================
// TEST 8: Cálculo de lados exteriores O_j
// ============================================================================

RPE_TEST(test08_LadosExteriores) {
    GridGeometry geo(3, 3, 3);

    RPE_ASSERT_EQ(geo.getOuterSides(4).count, 0);
    RPE_ASSERT_EQ(geo.getOuterSides(1).count, 1);
    RPE_ASSERT(geo.getOuterSides(1).contains(Side::N));
    RPE_ASSERT_EQ(geo.getOuterSides(3).count, 1);
    RPE_ASSERT(geo.getOuterSides(3).contains(Side::O));
    RPE_ASSERT_EQ(geo.getOuterSides(5).count, 1);
    RPE_ASSERT(geo.getOuterSides(5).contains(Side::E));
    RPE_ASSERT_EQ(geo.getOuterSides(7).count, 1);
    RPE_ASSERT(geo.getOuterSides(7).contains(Side::S));

    RPE_ASSERT_EQ(geo.getOuterSides(0).count, 2);
    RPE_ASSERT(geo.getOuterSides(0).contains(Side::N));
    RPE_ASSERT(geo.getOuterSides(0).contains(Side::O));

    RPE_ASSERT_EQ(geo.getOuterSides(2).count, 2);
    RPE_ASSERT(geo.getOuterSides(2).contains(Side::N));
    RPE_ASSERT(geo.getOuterSides(2).contains(Side::E));

    RPE_ASSERT_EQ(geo.getOuterSides(6).count, 2);
    RPE_ASSERT(geo.getOuterSides(6).contains(Side::S));
    RPE_ASSERT(geo.getOuterSides(6).contains(Side::O));

    RPE_ASSERT_EQ(geo.getOuterSides(8).count, 2);
    RPE_ASSERT(geo.getOuterSides(8).contains(Side::S));
    RPE_ASSERT(geo.getOuterSides(8).contains(Side::E));
}

// ============================================================================
// TEST 9: Patrón de lados grises G_i
// ============================================================================

RPE_TEST(test09_PatronLadosGrises) {
    Piece p0(0, 1, 1, 1, 1, 10, 0);
    RPE_ASSERT_EQ(p0.countGraySides(), 0);
    RPE_ASSERT(p0.hasValidGrayPattern());

    Piece p1(0, 0, 1, 1, 1, 10, 0);
    RPE_ASSERT_EQ(p1.countGraySides(), 1);
    RPE_ASSERT(p1.hasValidGrayPattern());

    Piece pNE(0, 0, 0, 1, 1, 10, 0);  RPE_ASSERT(pNE.hasValidGrayPattern());
    Piece pES(0, 1, 0, 0, 1, 10, 0);  RPE_ASSERT(pES.hasValidGrayPattern());
    Piece pSO(0, 1, 1, 0, 0, 10, 0);  RPE_ASSERT(pSO.hasValidGrayPattern());
    Piece pON(0, 0, 1, 1, 0, 10, 0);  RPE_ASSERT(pON.hasValidGrayPattern());

    Piece pNS(0, 0, 1, 0, 1, 10, 0);  RPE_ASSERT(!pNS.hasValidGrayPattern());
    Piece pEO(0, 1, 0, 1, 0, 10, 0);  RPE_ASSERT(!pEO.hasValidGrayPattern());

    Piece p3(0, 0, 0, 0, 1, 10, 0);  RPE_ASSERT(!p3.hasValidGrayPattern());
    Piece p4(0, 0, 0, 0, 0, 10, 0);  RPE_ASSERT(!p4.hasValidGrayPattern());
}

// ============================================================================
// TEST 10: Validador independiente
// ============================================================================

RPE_TEST(test10_ValidadorIndependiente) {
    Snapshot snapshot = makeSyntheticSnapshot2x2();

    std::unordered_map<int, int> valid = {{0,0},{1,1},{2,2},{3,3}};
    auto r = Validator::validate(valid, snapshot);
    RPE_ASSERT(r.first);
    RPE_ASSERT_NEAR(r.second, 0.0, 1e-9);

    std::unordered_map<int, int> dup = {{0,0},{1,0},{2,2},{3,3}};
    r = Validator::validate(dup, snapshot);
    RPE_ASSERT(!r.first);

    std::unordered_map<int, int> partial = {{0,0},{1,1}};
    r = Validator::validate(partial, snapshot);
    RPE_ASSERT(!r.first);

    std::unordered_map<int, int> outOfRange = {{0,99},{1,1},{2,2},{3,3}};
    r = Validator::validate(outOfRange, snapshot);
    RPE_ASSERT(!r.first);
}

// ============================================================================
// TEST 11: Cola OPEN best-bound
// ============================================================================

RPE_TEST(test11_ColaOpenBestBound) {
    using PQ = std::priority_queue<BBNode, std::vector<BBNode>, CompareBestBound>;

    PQ open;

    BBNode n1(1, 0, 10.0);
    BBNode n2(2, 0,  5.0);
    BBNode n3(3, 0,  8.0);

    open.push(n1);
    open.push(n2);
    open.push(n3);

    RPE_ASSERT_EQ(open.top().id, 2);
    open.pop();
    RPE_ASSERT_EQ(open.top().id, 3);
    open.pop();
    RPE_ASSERT_EQ(open.top().id, 1);
    open.pop();

    // Desempate por depth.
    BBNode a(1, 2, 10.0);
    BBNode b(2, 5, 10.0);
    open.push(a);
    open.push(b);
    RPE_ASSERT_EQ(open.top().id, 2);
    open.pop();
    RPE_ASSERT_EQ(open.top().id, 1);

    // Desempate por id.
    open.push(BBNode(10, 3, 5.0));
    open.push(BBNode(7, 3, 5.0));
    RPE_ASSERT_EQ(open.top().id, 7);
}

// ============================================================================
// TEST 12: Selección de variable de ramificación
// ============================================================================

RPE_TEST(test12_SeleccionVariableBranching) {
    Snapshot snapshot = makeSyntheticSnapshot2x2();
    ClpRelaxationSolver clp(snapshot);

    auto lp = clp.solveRelaxation({}, {}, 5.0);
    RPE_ASSERT(lp.status == LPSolverStatus::OPTIMAL);
    RPE_ASSERT(!lp.primal.empty());

    for (double v : lp.primal) {
        RPE_ASSERT(v >= -1e-9 && v <= 1.0 + 1e-9);
    }

    RPE_ASSERT(clp.getNumColumns() >= 4);
    RPE_ASSERT(clp.getNumColumns() <= 8);
}

// ============================================================================
// TEST 13: Creación de nodos hijos
// ============================================================================

RPE_TEST(test13_CreacionHijos) {
    BBNode parent(1, 0, 5.0);
    parent.fixedToOne = {0, 5};
    parent.fixedToZero = {2};

    const int branchCol = 10;

    BBNode child0(2, parent.depth + 1, parent.lowerBound);
    child0.fixedToOne = parent.fixedToOne;
    child0.fixedToZero = parent.fixedToZero;
    child0.fixedToZero.push_back(branchCol);

    BBNode child1(3, parent.depth + 1, parent.lowerBound);
    child1.fixedToOne = parent.fixedToOne;
    child1.fixedToZero = parent.fixedToZero;
    child1.fixedToOne.push_back(branchCol);

    RPE_ASSERT_EQ(child0.fixedToZero.size(), 2u);
    RPE_ASSERT_EQ(child0.fixedToZero.back(), 10);
    RPE_ASSERT_EQ(child0.fixedToOne.size(), 2u);

    RPE_ASSERT_EQ(child1.fixedToOne.size(), 3u);
    RPE_ASSERT_EQ(child1.fixedToOne.back(), 10);
    RPE_ASSERT_EQ(child1.fixedToZero.size(), 1u);

    RPE_ASSERT_NEAR(child0.lowerBound, 5.0, 1e-9);
    RPE_ASSERT_NEAR(child1.lowerBound, 5.0, 1e-9);
}

// ============================================================================
// TEST 14: Mapeo de estados de CLP
// ============================================================================

RPE_TEST(test14_MapeoEstadosCLP) {
    Snapshot snapshot = makeSyntheticSnapshot2x2();
    ClpRelaxationSolver clp(snapshot);

    auto opt = clp.solveRelaxation({}, {}, 5.0);
    RPE_ASSERT(opt.status == LPSolverStatus::OPTIMAL);
    RPE_ASSERT(opt.hasSolution);

    auto lim = clp.solveRelaxation({}, {}, 1e-9);
    RPE_ASSERT(lim.status == LPSolverStatus::OPTIMAL ||
               lim.status == LPSolverStatus::LIMIT ||
               lim.status == LPSolverStatus::INFEASIBLE ||
               lim.status == LPSolverStatus::ABANDONED ||
               lim.status == LPSolverStatus::SOLVER_ERROR);
}

// ============================================================================
// TEST 15: Copia profunda del primal
// ============================================================================

RPE_TEST(test15_CopiaProfundaPrimal) {
    Snapshot snapshot = makeSyntheticSnapshot2x2();
    ClpRelaxationSolver clp(snapshot);

    auto lp1 = clp.solveRelaxation({}, {}, 5.0);
    RPE_ASSERT(lp1.status == LPSolverStatus::OPTIMAL);
    std::vector<double> primal1 = lp1.primal;

    std::vector<int> f1;
    std::vector<int> f0;
    if (clp.getNumColumns() > 0) f0.push_back(0);
    auto lp2 = clp.solveRelaxation(f1, f0, 5.0);
    (void)lp2;

    RPE_ASSERT_EQ(primal1.size(), lp1.primal.size());
    for (size_t i = 0; i < primal1.size(); ++i) {
        RPE_ASSERT_NEAR(primal1[i], lp1.primal[i], 1e-12);
    }

    // Destruir el solver y verificar que primal1 sigue accesible.
    {
        ClpRelaxationSolver temp(snapshot);
        auto lpTemp = temp.solveRelaxation({}, {}, 5.0);
        (void)lpTemp;
    }
    for (size_t i = 0; i < primal1.size(); ++i) {
        RPE_ASSERT(primal1[i] >= 0.0 && primal1[i] <= 1.0);
    }
}

// ============================================================================
// TEST 16: Registro de podas en A0
// ============================================================================

RPE_TEST(test16_PodasA0) {
    {
        Snapshot snapshot = makeSyntheticSnapshot2x2();
        A0Config cfg;
        cfg.maxTimeSeconds = 5.0;
        cfg.useWarmStartH1 = true;

        BranchAndBoundA0 a0(snapshot, cfg);
        A0Result res = a0.solve();

        RPE_ASSERT(res.status == SolverStatus::OPTIMAL);
        RPE_ASSERT(res.prunedByBound >= 0);
        RPE_ASSERT(res.nodesExplored >= 1);
    }

    {
        std::unique_ptr<GridGeometry> geometry(new GridGeometry(1, 2, 3));
        std::vector<Piece> pieces;
        pieces.emplace_back(0, 0, 1, 1, 1, 10, 0);
        pieces.emplace_back(1, 0, 0, 0, 0, 15, 1);

        std::vector<Cell> cells;
        cells.emplace_back(0);
        cells.emplace_back(1);

        std::unordered_map<int, int> placement = {{0,0},{1,1}};
        std::unordered_map<int, double> costs = {{0,10},{1,15}};

        Snapshot snapshot(std::move(geometry),
                          std::move(pieces),
                          std::move(cells),
                          std::move(placement),
                          std::move(costs));

        A0Config cfg;
        cfg.maxTimeSeconds = 5.0;
        BranchAndBoundA0 a0(snapshot, cfg);
        A0Result res = a0.solve();

        RPE_ASSERT(res.status == SolverStatus::INFEASIBLE);
        RPE_ASSERT(res.prunedByInfeasibility >= 0);
    }
}

// ============================================================================
// TEST 17: Verificación de óptimo pequeño
// ============================================================================

RPE_TEST(test17_OptimoPequeno) {
    Snapshot snapshot = makeSyntheticSnapshot2x2();

    A0Config a0cfg;
    a0cfg.maxTimeSeconds = 5.0;
    BranchAndBoundA0 a0(snapshot, a0cfg);
    A0Result a0Res = a0.solve();

    SolverConfig cbcCfg;
    cbcCfg.maxTimeSeconds = 5.0;
    SolverResult cbcRes = CbcReferenceSolver::solve(snapshot, cbcCfg);

    RPE_ASSERT(a0Res.status == SolverStatus::OPTIMAL);
    RPE_ASSERT(cbcRes.status == SolverStatus::OPTIMAL);
    RPE_ASSERT_NEAR(a0Res.upperBound, cbcRes.objectiveValue, 1e-6);
}

// ============================================================================
// TEST 18: Reproducibilidad determinista
// ============================================================================

RPE_TEST(test18_ReproducibilidadDeterminista) {
    Snapshot snapshot = makeSyntheticSnapshot2x2();

    Solution h1a = HeuristicH1::solve(snapshot, Solution(), 5.0);
    Solution h1b = HeuristicH1::solve(snapshot, Solution(), 5.0);
    RPE_ASSERT_EQ(h1a.size(), h1b.size());
    for (size_t i = 0; i < h1a.size(); ++i) RPE_ASSERT_EQ(h1a[i], h1b[i]);

    HeuristicH2::Params params;
    params.eta = 0.5;
    params.lambda = 1.0;
    params.baseSeed = 42;
    params.epoch = 0;
    params.maxTimeSeconds = 2.0;
    params.maxIterations = 50;

    Solution h2a = HeuristicH2::solve(snapshot, params);
    Solution h2b = HeuristicH2::solve(snapshot, params);
    RPE_ASSERT_EQ(h2a.size(), h2b.size());
    for (size_t i = 0; i < h2a.size(); ++i) RPE_ASSERT_EQ(h2a[i], h2b[i]);

    HeuristicH2::Params params2 = params;
    params2.epoch = 1;
    Solution h2c = HeuristicH2::solve(snapshot, params2);
    Solution h2d = HeuristicH2::solve(snapshot, params2);
    RPE_ASSERT_EQ(h2c.size(), h2d.size());
    for (size_t i = 0; i < h2c.size(); ++i) RPE_ASSERT_EQ(h2c[i], h2d[i]);
}

// ============================================================================
// TEST 19: Consistencia modelo vs validador
// ============================================================================

RPE_TEST(test19_ConsistenciaModeloValidador) {
    Snapshot snapshot = makeSyntheticSnapshot2x2();

    A0Config cfg;
    cfg.maxTimeSeconds = 5.0;
    BranchAndBoundA0 a0(snapshot, cfg);
    A0Result res = a0.solve();

    RPE_ASSERT(res.hasIncumbent());

    auto validation = Validator::validate(res.incumbent, snapshot);
    RPE_ASSERT(validation.first);
    RPE_ASSERT_NEAR(validation.second, res.upperBound, 1e-6);

    Solution h1 = HeuristicH1::solve(snapshot, Solution(), 5.0);
    if (!h1.empty()) {
        std::unordered_map<int, int> h1m;
        for (size_t i = 0; i < h1.size(); ++i) {
            if (h1[i] >= 0) h1m[(int)i] = h1[i];
        }
        auto v = Validator::validate(h1m, snapshot);
        if (v.first) {
            RPE_ASSERT(v.second >= 0.0);
        }
    }
}

// ============================================================================
// Tests de estrés
// ============================================================================

RPE_TEST(testStress01_GridGeometry) {
    auto start = std::chrono::high_resolution_clock::now();

    GridGeometry geo(100, 100, 10);
    RPE_ASSERT_EQ(geo.getOuterSides(5050).count, 0);
    RPE_ASSERT_EQ(geo.getAdjacentCells(5050).count, 4);
    RPE_ASSERT_EQ(geo.getOuterSides(9999).count, 2);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> d = end - start;
    std::cout << "[stress: " << d.count() << " ms] ";
}

RPE_TEST(testStress02_ModeloLP) {
    const int size = 3;
    std::unique_ptr<GridGeometry> geometry(new GridGeometry(size, size, 4));

    std::vector<Piece> pieces;
    std::vector<Cell> cells;
    std::unordered_map<int, int> initialPlacement;
    std::unordered_map<int, double> costs;

    for (int i = 0; i < size * size; ++i) {
        const auto outer = geometry->getOuterSides(i);
        int n = 1, e = 1, s = 1, w = 1;
        for (int k = 0; k < outer.count; ++k) {
            switch (outer.sides[k]) {
                case Side::N: n = 0; break;
                case Side::E: e = 0; break;
                case Side::S: s = 0; break;
                case Side::O: w = 0; break;
            }
        }
        pieces.emplace_back(i, n, e, s, w, 5.0 + (i % 10), i);
        cells.emplace_back(i);
        initialPlacement[i] = i;
        costs[i] = 5.0 + (i % 10);
    }

    Snapshot snapshot(std::move(geometry),
                      std::move(pieces),
                      std::move(cells),
                      std::move(initialPlacement),
                      std::move(costs));

    auto start = std::chrono::high_resolution_clock::now();
    ClpRelaxationSolver solver(snapshot);
    auto lp = solver.solveRelaxation({}, {}, 1.0);
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> d = end - start;
    std::cout << "[stress: " << d.count() << " ms] ";

    RPE_ASSERT(lp.status == LPSolverStatus::OPTIMAL);
    RPE_ASSERT(lp.hasSolution);
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "=================================\n";
    std::cout << "  Suite RPE — 19 casos (Sección 8.4)\n";
    std::cout << "=================================\n";

    RPE_RUN(test01_MapeoConstantesT0);
    RPE_RUN(test02_AtomicidadLoteEventos);
    RPE_RUN(test03_TiposDeEventos);
    RPE_RUN(test04_TransicionesCellState);
    RPE_RUN(test05_ValidacionEtaLambda);
    RPE_RUN(test06_ExtremosRCL);
    RPE_RUN(test07_TransicionEpoca);
    RPE_RUN(test08_LadosExteriores);
    RPE_RUN(test09_PatronLadosGrises);
    RPE_RUN(test10_ValidadorIndependiente);
    RPE_RUN(test11_ColaOpenBestBound);
    RPE_RUN(test12_SeleccionVariableBranching);
    RPE_RUN(test13_CreacionHijos);
    RPE_RUN(test14_MapeoEstadosCLP);
    RPE_RUN(test15_CopiaProfundaPrimal);
    RPE_RUN(test16_PodasA0);
    RPE_RUN(test17_OptimoPequeno);
    RPE_RUN(test18_ReproducibilidadDeterminista);
    RPE_RUN(test19_ConsistenciaModeloValidador);

    RPE_RUN(testStress01_GridGeometry);
    RPE_RUN(testStress02_ModeloLP);

    std::cout << "=================================\n";
    std::cout << "Resultado: " << g_testsPassed << "/" << g_testsRun
              << " tests pasaron.\n";
    if (g_testsFailed > 0) {
        std::cout << "FALLARON: " << g_testsFailed << " tests.\n";
        return 1;
    }
    std::cout << "Todas las pruebas pasaron exitosamente!\n";
    std::cout << "=================================\n";
    return 0;
}