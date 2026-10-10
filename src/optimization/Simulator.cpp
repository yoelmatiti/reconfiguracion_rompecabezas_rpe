#include "optimization/Simulator.h"

#include "optimization/A0Strategy.h"
#include "optimization/H1Strategy.h"
#include "optimization/H2Strategy.h"
#include "optimization/RStrategy.h"
#include "optimization/HeuristicaH2.h"
#include "domain/Validator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace optimization {

// ============================================================================
// Constructor
// ============================================================================

Simulator::Simulator(domain::Grid grid,
                     domain::EpochCommands epochCommands,
                     double graspEta,
                     double graspLambda,
                     SolverType solverType,
                     bool runAllSolvers)
    : grid_(std::move(grid)),
      epochCommands_(std::move(epochCommands)),
      graspEta_(graspEta),
      graspLambda_(graspLambda),
      solverType_(solverType),
      runAllSolvers_(runAllSolvers),
      baseSeed_(12345ULL),
      maxIterations_(100),
      totalEpochs_(0),
      solvedEpochs_(0) {

    if (!std::isfinite(graspEta_) || graspEta_ < 0.0 || graspEta_ > 1.0) {
        throw std::invalid_argument("graspEta debe estar en [0, 1]");
    }
    if (!std::isfinite(graspLambda_) || graspLambda_ < 0.0) {
        throw std::invalid_argument("graspLambda debe ser no negativo");
    }

    results_.reserve(epochCommands_.size() + 1);
    const auto& pieces = grid_.getPieces();
    prevSolution_ = createEmptySolution(static_cast<int>(pieces.size()));
    totalEpochs_ = epochCommands_.size() + 1;
}

// ============================================================================
// Bucle principal
// ============================================================================

std::vector<EpochResult> Simulator::run() {
    std::cout << "Iniciando simulación RPE..." << std::endl;
    std::cout << "Total de épocas a procesar: " << totalEpochs_ << std::endl;
    std::cout << "Modo: "
              << (runAllSolvers_ ? "TODOS los solvers (Strategy)"
                                  : "Solver único (Strategy)")
              << std::endl;

    // --- ÉPOCA 0: estado inicial SIN comandos ---
    std::cout << "Procesando época 0 (estado inicial, sin comandos)..."
              << std::endl;

    EpochResult epoch0 = processEpoch(0, 0);
    if (epoch0.isValid) {
        prevSolution_ = epoch0.solution;
        solvedEpochs_++;
    } else {
        std::cerr << "Advertencia: La época 0 no produce estado factible."
                  << std::endl;
    }
    results_.push_back(std::move(epoch0));

    // --- ÉPOCAS 1, 2, ...: aplicar comandos y resolver ---
    int logicalEpoch = 1;
    for (const auto& epochPair : epochCommands_) {
        const int timestamp = epochPair.first;
        const auto& commands = epochPair.second;

        std::cout << "Procesando época " << logicalEpoch
                  << " (timestamp " << timestamp << ", "
                  << commands.size() << " comandos)..." << std::endl;

        BatchResult batch = applyCommandsAtomically(commands);
        if (!batch.success) {
            std::cerr << "  - Lote rechazado atómicamente: "
                      << batch.errorMsg << std::endl;
            std::cerr << "  - Comandos aplicados antes del fallo: "
                      << batch.appliedCount << " de " << commands.size()
                      << std::endl;
            EpochResult failed(logicalEpoch, Solution(), 0.0, false, 0.0);
            failed.timestamp = timestamp;
            results_.push_back(std::move(failed));
            logicalEpoch++;
            continue;
        }

        EpochResult result = processEpoch(logicalEpoch, timestamp);
        if (result.isValid) {
            prevSolution_ = result.solution;
            solvedEpochs_++;
        } else {
            std::cerr << "  - Época " << logicalEpoch
                      << " (timestamp " << timestamp
                      << ") resultó infactible." << std::endl;
        }
        results_.push_back(std::move(result));
        logicalEpoch++;
    }

    std::cout << "Simulación finalizada. Épocas resueltas: "
              << solvedEpochs_ << "/" << results_.size() << std::endl;

    return std::move(results_);
}

// ============================================================================
// Aplicación atómica de comandos
// ============================================================================

BatchResult Simulator::applyCommandsAtomically(
    const domain::CommandBatch& commands) {

    BatchResult result;
    result.success = false;
    result.appliedCount = 0;
    result.failedAt = 0;

    if (commands.empty()) {
        result.success = true;
        return result;
    }

    // 1. Copia profunda del grid actual.
    domain::Grid gridCopy = grid_.deepCopy();

    // 2. Aplicar cada comando sobre la copia.
    for (size_t i = 0; i < commands.size(); ++i) {
        const auto& cmd = commands[i];
        const bool ok = cmd->execute(gridCopy);

        if (!ok) {
            result.success = false;
            result.appliedCount = i;
            result.failedAt = i;
            result.errorMsg = "Comando falló: " + cmd->toString();
            return result;  // Rollback
        }
        result.appliedCount++;
    }

    // 3. Commit
    grid_ = std::move(gridCopy);
    result.success = true;
    return result;
}

// ============================================================================
// Factory de estrategias
// ============================================================================

std::unique_ptr<PlacementStrategy> Simulator::makeStrategy(SolverType type) const {
    switch (type) {
        case SolverType::A0:
            return std::unique_ptr<PlacementStrategy>(new A0Strategy());
        case SolverType::H1:
            return std::unique_ptr<PlacementStrategy>(new H1Strategy());
        case SolverType::H2:
            return std::unique_ptr<PlacementStrategy>(new H2Strategy());
        case SolverType::R:
            return std::unique_ptr<PlacementStrategy>(new RStrategy());
    }
    return nullptr;
}

// ============================================================================
// Construcción de RunConfig
// ============================================================================

RunConfig Simulator::makeRunConfig(int epoch, int timeLimitMs) const {
    RunConfig cfg;
    cfg.maxTimeSeconds      = timeLimitMs / 1000.0;
    cfg.baseSeed            = baseSeed_;
    cfg.epoch               = epoch;
    cfg.graspEta            = graspEta_;
    cfg.graspLambda         = graspLambda_;
    cfg.maxIterations       = maxIterations_;
    cfg.epsilonInt          = 1e-4;
    cfg.epsilonBound        = 1e-8;
    cfg.logLevel            = 0;
    cfg.useRepairHeuristic  = true;
    cfg.useWarmStartH1      = true;
    return cfg;
}

// ============================================================================
// Procesamiento de época
// ============================================================================

EpochResult Simulator::processEpoch(int epoch, int timestamp) {
    std::unique_ptr<domain::Snapshot> snapshot = grid_.createSnapshot();
    if (!snapshot) {
        return EpochResult(epoch, Solution(), 0.0, false, 0.0);
    }

    const auto& pieces = snapshot->getPieces();
    const size_t numPieces = pieces.size();
    const int timeLimitMs = 1000;

    SolveResult best;
    if (runAllSolvers_) {
        best = selectBestResult(*snapshot, timeLimitMs, epoch);
    } else {
        std::unique_ptr<PlacementStrategy> strat = makeStrategy(solverType_);
        if (!strat) {
            return EpochResult(epoch, Solution(), 0.0, false, 0.0);
        }
        RunConfig cfg = makeRunConfig(epoch, timeLimitMs);
        best = strat->solve(*snapshot, cfg);
    }

    if (!best.hasSolution() || best.solution.size() != numPieces) {
        std::cerr << "  - Ningún solver encontró solución factible." << std::endl;
        return EpochResult(epoch, Solution(), 0.0, false, 0.0);
    }

    // Costo de transición Z^t(x).
    double transitionCost = 0.0;
    for (size_t i = 0; i < numPieces; ++i) {
        const int currentCell = best.solution[i];
        const int prevCell = (i < prevSolution_.size()) ? prevSolution_[i] : -1;
        if (currentCell >= 0 && prevCell >= 0 && currentCell != prevCell) {
            transitionCost += pieces[i].getCost();
        }
    }

    EpochResult result(epoch, std::move(best.solution),
                       best.executionTimeSeconds,
                       true, best.objective);
    result.timestamp = timestamp;
    result.solverName = best.methodName;
    result.feasible = true;
    result.isValid = true;
    result.objectiveValue = best.objective;

    return result;
}

// ============================================================================
// Selección del mejor resultado (modo exhaustivo)
// ============================================================================

SolveResult Simulator::selectBestResult(const domain::Snapshot& snapshot,
                                         int timeLimitMs,
                                         int epoch) {
    SolveResult best;
    best.status = SolveStatus::NO_SOLUTION;
    best.methodName = "";

    const SolverType solvers[4] = {
        SolverType::A0,
        SolverType::H1,
        SolverType::H2,
        SolverType::R
    };

    for (SolverType st : solvers) {
        try {
            std::unique_ptr<PlacementStrategy> strat = makeStrategy(st);
            if (!strat) continue;

            RunConfig cfg = makeRunConfig(epoch, timeLimitMs);
            SolveResult res = strat->solve(snapshot, cfg);

            if (!res.hasSolution()) continue;

            if (best.status == SolveStatus::NO_SOLUTION ||
                isBetterResult(res, best, cfg.epsilonBound)) {
                best = std::move(res);
            }
        } catch (const std::exception& e) {
            std::cerr << "  - Solver " << static_cast<int>(st)
                      << " lanzó excepción: " << e.what() << std::endl;
        }
    }

    return best;
}

// ============================================================================
// Comparación con desempate lexicográfico
// ============================================================================

bool Simulator::isBetterResult(const SolveResult& a,
                                const SolveResult& b,
                                double epsilonBound) {
    if (a.objective < b.objective - epsilonBound) return true;
    if (b.objective < a.objective - epsilonBound) return false;

    const size_t n = std::min(a.solution.size(), b.solution.size());
    for (size_t i = 0; i < n; ++i) {
        if (a.solution[i] < b.solution[i]) return true;
        if (a.solution[i] > b.solution[i]) return false;
    }
    return false;
}

// ============================================================================
// Evaluación multi-semilla de GRASP (Sección 9)
// ============================================================================

GraspMultiSeedResult Simulator::runGraspMultiSeed(int numSeeds,
                                                    int timeLimitMs,
                                                    int epoch) const {
    GraspMultiSeedResult result;
    result.numSeeds = std::max(1, numSeeds);

    // Snapshot fresco del grid actual (NO modifica el estado interno).
    std::unique_ptr<domain::Snapshot> snapshot = grid_.createSnapshot();
    if (!snapshot) {
        return result;
    }

    const auto& pieces = snapshot->getPieces();
    const size_t numPieces = pieces.size();

    result.perSeed.reserve(static_cast<size_t>(result.numSeeds));

    for (int i = 0; i < result.numSeeds; ++i) {
        GraspSeedResult seedRes;
        seedRes.seed = baseSeed_ + static_cast<uint64_t>(i);
        seedRes.epoch = epoch;

        const auto t0 = std::chrono::steady_clock::now();

        try {
            HeuristicH2::Params params;
            params.eta = graspEta_;
            params.lambda = graspLambda_;
            params.baseSeed = seedRes.seed;
            params.epoch = static_cast<uint32_t>(epoch);
            params.maxTimeSeconds = timeLimitMs / 1000.0;
            params.maxIterations = maxIterations_;

            Solution sol = HeuristicH2::solve(*snapshot, params);

            seedRes.runtimeMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();

            if (!sol.empty() && sol.size() == numPieces) {
                std::unordered_map<int, int> mapSol;
                mapSol.reserve(numPieces);
                for (size_t k = 0; k < numPieces; ++k) {
                    if (sol[k] >= 0) {
                        mapSol[pieces[k].getId()] = sol[k];
                    }
                }
                auto val = domain::Validator::validate(mapSol, *snapshot);
                if (val.first) {
                    seedRes.objective = val.second;
                    seedRes.solution = std::move(sol);
                    seedRes.bestFound = true;
                    seedRes.feasibleConstructions = 1;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "  - GRASP seed " << seedRes.seed
                      << " falló: " << e.what() << std::endl;
        }

        result.perSeed.push_back(std::move(seedRes));
    }

    // ---- Consolidado estadístico ----
    double sumCost = 0.0;
    double sumSqCost = 0.0;
    double sumRuntime = 0.0;
    int feasibleCount = 0;

    for (const auto& r : result.perSeed) {
        sumRuntime += r.runtimeMs;
        if (r.bestFound) {
            sumCost += r.objective;
            sumSqCost += r.objective * r.objective;
            feasibleCount++;

            if (r.objective < result.bestCost) {
                result.bestCost = r.objective;
                result.bestSolution = r.solution;
                result.bestSeed = r.seed;
            }
            if (r.objective > result.worstCost) {
                result.worstCost = r.objective;
            }
        }
    }

    result.numFeasible = feasibleCount;
    result.avgRuntimeMs = (result.numSeeds > 0)
                          ? sumRuntime / result.numSeeds
                          : 0.0;

    if (feasibleCount > 0) {
        result.avgCost = sumCost / feasibleCount;
        const double variance = (sumSqCost / feasibleCount)
                                - (result.avgCost * result.avgCost);
        result.stdDevCost = std::sqrt(std::max(0.0, variance));
    } else {
        result.avgCost = std::numeric_limits<double>::infinity();
        result.worstCost = std::numeric_limits<double>::infinity();
    }

    return result;
}

// ============================================================================
// createEmptySolution
// ============================================================================

Solution Simulator::createEmptySolution(int numPieces) const {
    return Solution(numPieces, -1);
}

} // namespace optimization