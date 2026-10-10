#include "optimization/ClpRelaxationSolver.h"
#include <coin/ClpSimplex.hpp>
#include <coin/CoinPackedMatrix.hpp>

#include <algorithm>
#include <cassert>
#include <limits>
#include <stdexcept>
#include <chrono>

namespace optimization {

namespace {
    const double EPSILON_TIME = 0.001;
    const double INF = std::numeric_limits<double>::infinity();
}

// ============================================================================
// Constructor
// ============================================================================

ClpRelaxationSolver::ClpRelaxationSolver(const domain::Snapshot& snapshot)
    : clpSolver_(nullptr),
      numColumns_(0),
      numRows_(0),
      hasSolution_(false),
      lastObjective_(INF),
      isFirstSolve_(true) {
    
    // 1. Construir el modelo desde el Snapshot
    OptimizationModelData modelData;
    try {
        modelData = MatrixBuilder::buildModelData(snapshot);
    } catch (const std::exception& e) {
        throw std::runtime_error(
            std::string("Error construyendo el modelo LP: ") + e.what()
        );
    }
    
    // Un modelo sin variables no es un error de construcción: representa una
    // instancia infactible o vacía que debe resolverse como tal en A0.
    // Se preservan los datos del modelo y se deja que solveRelaxation() los
    // clasifique con un estado de infactibilidad.
    columnMetas_ = std::move(modelData.colToPairMap);
    objective_ = std::move(modelData.objective);
    rowLowerBounds_ = std::move(modelData.rowLowerBounds);
    rowUpperBounds_ = std::move(modelData.rowUpperBounds);
    
    numColumns_ = static_cast<int>(columnMetas_.size());
    numRows_ = static_cast<int>(rowLowerBounds_.size());
    
    // Si el modelo no tiene variables, no se inicializa CLP y el solver
    // responderá en solveRelaxation() con INFEASIBLE.
    if (numColumns_ == 0) {
        return;
    }
    
    // 2. Construir cotas base (con pistas aplicadas)
    // MatrixBuilder ya devuelve colLowerBounds/colUpperBounds con pistas aplicadas
    baseColLower_ = std::move(modelData.colLowerBounds);
    baseColUpper_ = std::move(modelData.colUpperBounds);
    
    // Validación defensiva
    assert(static_cast<int>(baseColLower_.size()) == numColumns_);
    assert(static_cast<int>(baseColUpper_.size()) == numColumns_);
    assert(static_cast<int>(objective_.size()) == numColumns_);
    assert(static_cast<int>(columnMetas_.size()) == numColumns_);
    
    // 4. Inicializar buffers de trabajo
    colLower_.assign(numColumns_, 0.0);
    colUpper_.assign(numColumns_, 1.0);
    lastPrimal_.assign(numColumns_, 0.0);
    
    // 5. Crear el solver CLP y cargar el modelo raíz
    clpSolver_ = new ClpSimplex();
    clpSolver_->setLogLevel(0);  // Silencioso
    
    // Nota: loadProblem copia internamente la matriz, así que podemos
    // liberar modelData.matrix después
    clpSolver_->loadProblem(
        *(modelData.matrix),
        baseColLower_.data(),
        baseColUpper_.data(),
        objective_.data(),
        rowLowerBounds_.data(),
        rowUpperBounds_.data()
    );
    
    // Liberar la matriz (CLP ya tiene su propia copia interna)
    modelData.matrix.reset();
    
    // 6. Dirección de optimización: MINIMIZAR
    clpSolver_->setOptimizationDirection(1.0);
    
    // 7. Construir tabla hash para mapeo (pieceId, cellId) → colIdx
    varMap_.reserve(numColumns_);
    for (int i = 0; i < numColumns_; ++i) {
        const uint64_t key = makeKey(columnMetas_[i].pieceId, 
                                      columnMetas_[i].cellId);
        varMap_[key] = i;
    }
}

// ============================================================================
// Destructor
// ============================================================================

ClpRelaxationSolver::~ClpRelaxationSolver() {
    delete clpSolver_;
    clpSolver_ = nullptr;
}

// ============================================================================
// Resolución de relajación LP
// ============================================================================

LPSolverResult ClpRelaxationSolver::solveRelaxation(
    const std::vector<int>& fixedToOne,
    const std::vector<int>& fixedToZero,
    double timeLimitSeconds) {
    
    LPSolverResult result;
    
    // Validación defensiva
    if (numColumns_ == 0) {
        result.status = LPSolverStatus::INFEASIBLE;
        result.message = "Modelo sin columnas: infeasible";
        return result;
    }
    
    if (clpSolver_ == nullptr) {
        result.status = LPSolverStatus::SOLVER_ERROR;
        result.message = "Solver CLP no inicializado";
        return result;
    }
    
    // 1. Resetear cotas a base (con pistas aplicadas)
    resetColumnBounds();
    
    // 2. Aplicar fijaciones del nodo
    applyFixings(fixedToOne, fixedToZero);
    
    // 3. Push de cotas a CLP (con arrays de tamaño EXACTO)
    pushBoundsToSolver();
    
    // 4. Configurar tiempo límite
    if (timeLimitSeconds > 0.0) {
        clpSolver_->setMaximumWallSeconds(timeLimitSeconds);
    }
    
    // 5. Resolver (initialDualSolve para la raíz, dual para nodos hijos)
    try {
        if (isFirstSolve_) {
            clpSolver_->initialDualSolve();
            isFirstSolve_ = false;
        } else {
            clpSolver_->dual();
        }
    } catch (const std::exception& e) {
        result.status = LPSolverStatus::SOLVER_ERROR;
        result.message = std::string("Excepción en CLP: ") + e.what();
        return result;
    } catch (...) {
        result.status = LPSolverStatus::SOLVER_ERROR;
        result.message = "Excepción desconocida en CLP";
        return result;
    }
    
    // 6. Clasificar resultado
    result.status = classifySolverStatus(timeLimitSeconds);
    result.iterations = clpSolver_->getIterationCount();
    
    // 7. Extraer resultados SOLO si optimalidad demostrada
    if (result.status == LPSolverStatus::OPTIMAL) {
        result.hasSolution = true;
        result.objective = clpSolver_->getObjValue();
        lastObjective_ = result.objective;
        hasSolution_ = true;
        
        // Copia profunda del primal (evita punteros colgantes)
        const double* primalPtr = clpSolver_->primalColumnSolution();
        if (primalPtr != nullptr) {
            // Copia directa al vector del resultado
            result.primal.assign(primalPtr, primalPtr + numColumns_);
            
            // Actualizar cache interno
            lastPrimal_ = result.primal;
        } else {
            // Sin primal: tratar como error
            result.status = LPSolverStatus::SOLVER_ERROR;
            result.message = "primalColumnSolution() retornó nullptr";
            result.hasSolution = false;
            hasSolution_ = false;
            result.objective = INF;
        }
    } else {
        // No óptimo: sin solución
        result.hasSolution = false;
        hasSolution_ = false;
        result.objective = INF;
        
        // Diagnóstico
        switch (result.status) {
            case LPSolverStatus::INFEASIBLE:
                result.message = "LP primal infactible";
                break;
            case LPSolverStatus::LIMIT:
                result.message = "Límite de iteraciones/tiempo alcanzado";
                break;
            case LPSolverStatus::ABANDONED:
                result.message = "CLP abandonó la optimización";
                break;
            default:
                result.message = "Estado desconocido";
                break;
        }
    }
    
    return result;
}

// ============================================================================
// Helpers privados
// ============================================================================

void ClpRelaxationSolver::resetColumnBounds() {
    // Copia de las cotas base (que incluyen pistas ya aplicadas)
    // Asegura que las columnas permanentemente prohibidas por pistas
    // permanezcan en [0,0].
    colLower_ = baseColLower_;
    colUpper_ = baseColUpper_;
}

void ClpRelaxationSolver::applyFixings(
    const std::vector<int>& fixedToOne,
    const std::vector<int>& fixedToZero) {
    
    // Aplicar fijaciones a 1
    for (int varIdx : fixedToOne) {
        if (varIdx < 0 || varIdx >= numColumns_) {
            continue;  // Ignorar índices inválidos
        }
        // No sobreescribir columnas permanentemente prohibidas
        if (baseColUpper_[varIdx] < 0.5) {
            continue;
        }
        colLower_[varIdx] = 1.0;
        colUpper_[varIdx] = 1.0;
    }
    
    // Aplicar fijaciones a 0
    for (int varIdx : fixedToZero) {
        if (varIdx < 0 || varIdx >= numColumns_) {
            continue;
        }
        colLower_[varIdx] = 0.0;
        colUpper_[varIdx] = 0.0;
    }
}

void ClpRelaxationSolver::pushBoundsToSolver() {
    // Validación defensiva: los buffers DEBEN tener tamaño exacto
    assert(static_cast<int>(colLower_.size()) == numColumns_);
    assert(static_cast<int>(colUpper_.size()) == numColumns_);
    
    // Push de cotas a CLP con arrays de tamaño completo
    // (evita Invalid Read de tamaño 8 en Valgrind)
    clpSolver_->chgColumnLower(colLower_.data());
    clpSolver_->chgColumnUpper(colUpper_.data());
}

LPSolverStatus ClpRelaxationSolver::classifySolverStatus(double timeLimitSeconds) {
    // Orden normativo del enunciado (sección 5.2, paso 5):
    // 1. isProvenOptimal
    // 2. isProvenPrimalInfeasible
    // 3. hitMaximumIterations / límite de tiempo
    // 4. isAbandoned
    // 5. Cualquier otro → SOLVER_ERROR
    
    if (clpSolver_->isProvenOptimal()) {
        return LPSolverStatus::OPTIMAL;
    }
    
    // Nota: solo primal infactible (no dual infactible)
    if (clpSolver_->isProvenPrimalInfeasible()) {
        return LPSolverStatus::INFEASIBLE;
    }
    
    // Límite de iteraciones o tiempo
    if (clpSolver_->hitMaximumIterations()) {
        return LPSolverStatus::LIMIT;
    }
    
    // Si CLP marcó abandono por límite temporal o por cualquier otra causa,
    // lo tratamos como límite en esta versión de la API de COIN-OR.
    if (timeLimitSeconds > 0.0 && clpSolver_->isAbandoned()) {
        return LPSolverStatus::LIMIT;
    }
    
    // CLP abandonó
    if (clpSolver_->isAbandoned()) {
        return LPSolverStatus::ABANDONED;
    }
    
    // Estado desconocido
    return LPSolverStatus::SOLVER_ERROR;
}

// ============================================================================
// Accesores
// ============================================================================

int ClpRelaxationSolver::getColumnIndex(int pieceId, int cellId) const {
    const uint64_t key = makeKey(pieceId, cellId);
    auto it = varMap_.find(key);
    return (it != varMap_.end()) ? it->second : -1;
}

const ColumnMeta& ClpRelaxationSolver::getColumnMeta(int colIdx) const {
    if (colIdx < 0 || colIdx >= numColumns_) {
        throw std::out_of_range("Índice de columna fuera de rango");
    }
    return columnMetas_[colIdx];
}

bool ClpRelaxationSolver::isPermanentlyFixedToZero(int colIdx) const {
    if (colIdx < 0 || colIdx >= numColumns_) {
        return false;
    }
    return baseColUpper_[colIdx] < 0.5;
}

} // namespace optimization