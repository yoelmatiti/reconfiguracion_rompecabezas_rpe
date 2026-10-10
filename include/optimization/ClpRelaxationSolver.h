#ifndef OPTIMIZATION_CLPRELAXATIONSOLVER_H
#define OPTIMIZATION_CLPRELAXATIONSOLVER_H

#include <vector>
#include <memory>
#include <unordered_map>
#include <cstdint>
#include "domain/Snapshot.h"
#include "optimization/SolverTypes.h"
#include "optimization/MatrixBuilder.h"

// Forward declarations de librerías de COIN-OR
class ClpSimplex;
class CoinPackedMatrix;

namespace optimization {

/**
 * @brief Adaptador envolvente de ClpSimplex optimizado para Branch & Bound.
 * 
 * Características:
 * - Carga el modelo raíz UNA SOLA VEZ en el constructor.
 * - Distingue entre columnas PERMANENTEMENTE prohibidas (pistas) y
 *   columnas TEMPORALMENTE fijadas (nodos B&B).
 * - Reutiliza la misma instancia de ClpSimplex sin recrearla.
 * - Realiza copia profunda de la solución primal antes de cualquier
 *   reoptimización (evita punteros colgantes de CLP).
 * 
 * Uso típico:
 *   ClpRelaxationSolver solver(snapshot);
 *   LPSolverResult lp = solver.solveRelaxation(fixedToOne, fixedToZero, 10.0);
 *   // lp.status, lp.objective, lp.primal son copias propias
 */
class ClpRelaxationSolver {
public:
    /**
     * @brief Constructor: construye el modelo raíz y carga CLP.
     * 
     * @param snapshot Snapshot inmutable del problema.
     * @throws std::runtime_error si el modelo no se puede construir.
     */
    explicit ClpRelaxationSolver(const domain::Snapshot& snapshot);
    
    /**
     * @brief Destructor: libera la memoria nativa de ClpSimplex.
     */
    ~ClpRelaxationSolver();

    // Inhabilitar copia (evita fugas y doble delete)
    ClpRelaxationSolver(const ClpRelaxationSolver&) = delete;
    ClpRelaxationSolver& operator=(const ClpRelaxationSolver&) = delete;

    /**
     * @brief Aplica las fijaciones del nodo y resuelve la relajación LP.
     * 
     * Estrategia:
     * 1. Resetear cotas a [0,1] (excepto columnas permanentemente prohibidas).
     * 2. Aplicar fijaciones del nodo.
     * 3. Resolver con dual() (raíz se resuelve con initialDualSolve() la
     *    primera vez).
     * 4. Clasificar resultado y hacer copia profunda del primal.
     * 
     * @param fixedToOne Índices de columnas fijadas a 1.
     * @param fixedToZero Índices de columnas fijadas a 0.
     * @param timeLimitSeconds Tiempo máximo de resolución.
     * @return LPSolverResult Resultado con copia propia del primal.
     */
    LPSolverResult solveRelaxation(const std::vector<int>& fixedToOne,
                                   const std::vector<int>& fixedToZero,
                                   double timeLimitSeconds);

    /**
     * @brief Obtiene el índice de columna para (pieceId, cellId).
     * @return Índice de columna, -1 si no existe.
     */
    int getColumnIndex(int pieceId, int cellId) const;

    /**
     * @brief Recupera los metadatos de una columna.
     * @throws std::out_of_range si colIdx no es válido.
     */
    const ColumnMeta& getColumnMeta(int colIdx) const;

    // Accesores O(1)
    int getNumColumns() const noexcept { return numColumns_; }
    int getNumRows() const noexcept { return numRows_; }
    const std::unordered_map<uint64_t, int>& getVarMap() const noexcept { 
        return varMap_; 
    }
    bool hasSolution() const noexcept { return hasSolution_; }
    const std::vector<double>& getLastPrimal() const noexcept { 
        return lastPrimal_; 
    }
    double getLastObjective() const noexcept { return lastObjective_; }
    
    /**
     * @brief Verifica si una columna está permanentemente prohibida por pista.
     */
    bool isPermanentlyFixedToZero(int colIdx) const;

private:
    /**
     * @brief Genera clave de 64 bits sin colisión a partir de dos IDs.
     */
    static inline uint64_t makeKey(int pieceId, int cellId) noexcept {
        return (static_cast<uint64_t>(pieceId) << 32) | 
               (static_cast<uint32_t>(cellId) & 0xFFFFFFFFULL);
    }

    /**
     * @brief Restablece las cotas de columnas a [0,1] excepto las
     * permanentemente prohibidas por pistas (que quedan en [0,0]).
     */
    void resetColumnBounds();

    /**
     * @brief Aplica las fijaciones del nodo B&B sobre colLower_/colUpper_.
     * 
     * @note NO toca columnas permanentemente fijadas a 0.
     */
    void applyFixings(const std::vector<int>& fixedToOne,
                      const std::vector<int>& fixedToZero);

    /**
     * @brief Clasifica el estado del solver CLP según el orden normativo.
     */
    LPSolverStatus classifySolverStatus(double timeLimitSeconds);

    /**
     * @brief Aplica todas las cotas al solver CLP.
     * 
     * @note Usa chgColumnLower/chgColumnUpper con arrays de tamaño
     *       EXACTO numColumns_ para evitar Invalid Read.
     */
    void pushBoundsToSolver();

    // --- Miembros ---
    ClpSimplex* clpSolver_;                ///< Puntero nativo al motor CLP
    int numColumns_;                       ///< Total de columnas x_{ij}
    int numRows_;                          ///< Total de restricciones
    
    std::vector<double> objective_;        ///< Coeficientes c_{ij}^t
    std::vector<double> rowLowerBounds_;
    std::vector<double> rowUpperBounds_;
    
    std::vector<ColumnMeta> columnMetas_;  ///< colIdx → metadata
    std::unordered_map<uint64_t, int> varMap_;  ///< (pieceId,cellId) → colIdx
    
    bool hasSolution_;                     ///< Cache de solución
    double lastObjective_;                 ///< Cache de objetivo
    std::vector<double> lastPrimal_;       ///< Cache del primal
    
    // Buffers pre-reservados (tamaño SIEMPRE == numColumns_)
    std::vector<double> colLower_;         ///< Cotas inferiores actuales
    std::vector<double> colUpper_;         ///< Cotas superiores actuales
    std::vector<double> baseColLower_;     ///< Cotas base (con pistas aplicadas)
    std::vector<double> baseColUpper_;     ///< Cotas base (con pistas aplicadas)
    
    bool isFirstSolve_;                    ///< Flag para usar initialDualSolve()
};

} // namespace optimization

#endif // OPTIMIZATION_CLPRELAXATIONSOLVER_H