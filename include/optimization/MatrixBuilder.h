#ifndef OPTIMIZATION_MATRIXBUILDER_H
#define OPTIMIZATION_MATRIXBUILDER_H

#include <vector>
#include <memory>
#include <coin/CoinPackedMatrix.hpp>
#include "domain/Snapshot.h"
#include "optimization/SolverTypes.h"

namespace optimization {

/**
 * @brief Contenedor del modelo LP construido a partir de un Snapshot.
 * 
 * Almacena la matriz dispersa CSC, los metadatos de columnas y todas
 * las cotas y coeficientes objetivo necesarios para cargar en CLP/Cbc.
 * 
 * IMPORTANTE: Los vectores de cotas tienen tamaño EXACTO numVars y numRows.
 * Esto evita Invalid Read/Write en Valgrind al pasarlos a CLP.
 */
struct OptimizationModelData {
    std::unique_ptr<CoinPackedMatrix> matrix;  ///< Matriz dispersa CSC
    std::vector<ColumnMeta> colToPairMap;      ///< colIdx → ColumnMeta
    std::vector<double> objective;             ///< Coeficientes c_ij^t
    std::vector<double> rowLowerBounds;        ///< Cotas inferiores de filas (tamaño numRows)
    std::vector<double> rowUpperBounds;        ///< Cotas superiores de filas (tamaño numRows)
    std::vector<double> colLowerBounds;        ///< Cotas inferiores de columnas (tamaño numVars)
    std::vector<double> colUpperBounds;        ///< Cotas superiores de columnas (tamaño numVars)
    
    int numPieces;     ///< Número de piezas |I|
    int numCells;      ///< Número de celdas |J|
    int numVars;       ///< Número de variables (columnas)
    int numRows;       ///< Número de restricciones (filas)
    
    OptimizationModelData()
        : numPieces(0), numCells(0), numVars(0), numRows(0) {}
    
    /**
     * @brief Indica si el modelo contiene al menos una variable.
     */
    bool isValid() const {
        return matrix.get() != nullptr && numVars > 0;
    }
};

/**
 * @brief Constructor de la matriz LP del problema RPE.
 * 
 * Genera una columna por cada par (i,j) con G_i == O_j, aplica el mapeo
 * κ(i,j) = i·|J| + j, y construye las restricciones (3b), (3c), (3f)-(3g).
 * 
 * Las pistas (3e) se aplican al final fijando cotas en [0,0].
 */
class MatrixBuilder {
public:
    /**
     * @brief Construye el modelo LP completo a partir de un Snapshot.
     * 
     * @param snapshot Snapshot inmutable de la época actual.
     * @return OptimizationModelData Modelo LP listo para CLP/Cbc.
     * @throws std::runtime_error si el snapshot es inconsistente.
     */
    static OptimizationModelData buildModelData(const domain::Snapshot& snapshot);

private:
    /**
     * @brief Verifica si la pieza puede ocupar la celda (3d).
     */
    static bool isStructurallyCompatible(
        const domain::Snapshot& snapshot,
        int pieceIdx,
        int cellIdx);
    
    /**
     * @brief Verifica si la pieza está prohibida por una pista (3e).
     */
    static bool isProhibitedByClue(
        const domain::Snapshot& snapshot,
        int pieceIdx,
        int cellIdx);
    
    /**
     * @brief Calcula κ(i,j) = i·|J| + j.
     */
    static int computeKappa(int pieceIdx, int cellIdx, int numCells);
    
    /**
     * @brief Obtiene el lado opuesto de un lado dado.
     */
    static domain::Side oppositeSide(domain::Side side);
};

} // namespace optimization

#endif // OPTIMIZATION_MATRIXBUILDER_H