#include "optimization/CbcReferenceSolver.h"
#include "domain/Validator.h"

#include <coin/CoinPackedMatrix.hpp>
#include <coin/OsiClpSolverInterface.hpp>
#include <coin/CbcModel.hpp>
#include <coin/CbcCutGenerator.hpp>
#include <coin/CglGomory.hpp>
#include <coin/CglKnapsackCover.hpp>
#include <coin/CglProbing.hpp>

#include <chrono>
#include <cmath>
#include <algorithm>

namespace optimization {

SolverResult CbcReferenceSolver::solve(const domain::Snapshot& snapshot, 
                                       const SolverConfig& config) {
    SolverResult result;
    const auto startTime = std::chrono::steady_clock::now();

    try {
        const auto& pieces = snapshot.getPieces();
        const size_t numPieces = pieces.size();

        // 1. Construcción de la matriz dispersa
        auto modelData = MatrixBuilder::buildModelData(snapshot);

        if (!modelData.matrix || modelData.matrix->getNumCols() == 0) {
            result.status = SolverStatus::INFEASIBLE;
            result.statusMessage = "Sin columnas con compatibilidad geométrica básica.";
            return result;
        }

        const auto& colToPairMap = modelData.colToPairMap;
        const auto& objective = modelData.objective;
        const auto& rowLowerBounds = modelData.rowLowerBounds;
        const auto& rowUpperBounds = modelData.rowUpperBounds;
        auto matrix = std::move(modelData.matrix);

        // 2. Inicializar el solver CLP a través de OSI
        OsiClpSolverInterface solver;
        solver.messageHandler()->setLogLevel(0);
        solver.setDblParam(OsiPrimalTolerance, config.primalTolerance);
        solver.setDblParam(OsiDualTolerance, config.dualTolerance);

        const int numColumns = static_cast<int>(colToPairMap.size());

        solver.loadProblem(*matrix, nullptr, nullptr, objective.data(),
                           rowLowerBounds.data(), rowUpperBounds.data());

        // Liberar la memoria de la matriz en C++ una vez transferida al solver
        matrix.reset();

        // Mapeo directo para búsqueda $O(1)$ de piezas por ID
        std::unordered_map<int, const domain::Piece*> pieceByIdMap;
        pieceByIdMap.reserve(numPieces);
        for (const auto& p : pieces) {
            pieceByIdMap[p.getId()] = &p;
        }

        // 3. Aplicar pistas (clues) y establecer integralidad de variables
        for (int k = 0; k < numColumns; ++k) {
            const auto& meta = colToPairMap[k];
            double ub = 1.0;

            const auto* clue = snapshot.getCellClue(meta.cellId);
            if (clue != nullptr) {
                auto it = pieceByIdMap.find(meta.pieceId);
                if (it != pieceByIdMap.end()) {
                    if (it->second->getColor(clue->first) != clue->second) {
                        ub = 0.0; // Incompatible con la pista dada
                    }
                }
            }

            solver.setColLower(k, 0.0);
            solver.setColUpper(k, ub);
            solver.setInteger(k);
        }

        solver.setObjSense(1.0); // Minimización

        // 4. Configurar e invocar CbcModel
        CbcModel model(solver);
        model.messageHandler()->setLogLevel(config.logLevel);

        if (config.maxTimeSeconds > 0.0) {
            model.setMaximumSeconds(config.maxTimeSeconds);
        }
        if (config.maxNodes > 0) {
            model.setMaximumNodes(config.maxNodes);
        }

        model.setIntegerTolerance(config.integerTolerance);

        if (!config.useHeuristics) {
            model.setNumberHeuristics(0);
        }

        // Instanciación de generadores de cortes garantizando vida durante branchAndBound
        CglGomory gomory;
        CglProbing probing;
        CglKnapsackCover knapsack;

        if (config.useCuts) {
            gomory.setLimit(30);
            model.addCutGenerator(&gomory, -1, "Gomory");

            probing.setMaxProbe(10);
            model.addCutGenerator(&probing, -1, "Probing");

            model.addCutGenerator(&knapsack, -1, "Knapsack");
        }

        // Resolver el modelo Branch-and-Cut
        model.branchAndBound();

        // 5. Métricas de salida
        const auto endTime = std::chrono::steady_clock::now();
        result.executionTimeSeconds = std::chrono::duration<double>(endTime - startTime).count();
        result.nodeCount = model.getNodeCount();
        result.iterationCount = model.getIterationCount();
        result.bestLowerBound = model.getBestPossibleObjValue();

        // Evaluación de estados de parada
        if (model.isProvenInfeasible()) {
            result.status = SolverStatus::INFEASIBLE;
            result.statusMessage = "Infactibilidad demostrada por Cbc.";
            return result;
        }

        if (model.isAbandoned()) {
            result.status = SolverStatus::SOLVER_ERROR;
            result.statusMessage = "Cbc abandonó la optimización debido a un error interno.";
            return result;
        }

        const double* bestSolution = model.bestSolution();
        if (bestSolution != nullptr) {
            // Inicializar el diccionario de solución de salida
            result.solution.reserve(numPieces);
            for (const auto& p : pieces) {
                result.solution[p.getId()] = -1;
            }

            // Extraer las variables activas (>= 1.0 - tolerancia)
            const double threshold = 1.0 - config.integerTolerance;
            for (int k = 0; k < numColumns; ++k) {
                if (bestSolution[k] >= threshold) {
                    const auto& meta = colToPairMap[k];
                    result.solution[meta.pieceId] = meta.cellId;
                }
            }

            // Validar la solución frente a las reglas del dominio
            std::pair<bool, double> validation = domain::Validator::validate(result.solution, snapshot);
            const bool isValidSolution = validation.first;
            const double objValue = validation.second;
            if (!isValidSolution) {
                result.status = SolverStatus::INVALID;
                result.statusMessage = "La solución devuelta por Cbc no cumplió las reglas del dominio.";
                return result;
            }

            result.objectiveValue = objValue;
            result.status = model.isProvenOptimal() ? SolverStatus::OPTIMAL : SolverStatus::FEASIBLE;
            result.statusMessage = model.isProvenOptimal() ? "Solución óptima alcanzada." 
                                                           : "Solución factible encontrada.";
            return result;
        }

        if (config.maxTimeSeconds > 0.0 && model.getCurrentSeconds() >= config.maxTimeSeconds) {
            result.status = SolverStatus::NO_INCUMBENT;
            result.statusMessage = "Tiempo máximo agotado sin encontrar solución entera.";
        } else {
            result.status = SolverStatus::SOLVER_ERROR;
            result.statusMessage = "Finalizó la búsqueda sin solución factible.";
        }

    } catch (const std::exception& e) {
        result.status = SolverStatus::SOLVER_ERROR;
        result.statusMessage = std::string("Excepción en CbcReferenceSolver: ") + e.what();
    } catch (...) {
        result.status = SolverStatus::SOLVER_ERROR;
        result.statusMessage = "Excepción no identificada en CbcReferenceSolver.";
    }

    return result;
}

} // namespace optimization