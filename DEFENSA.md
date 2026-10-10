# DEFENSA.md

## Decisiones clave de diseño del proyecto RPE

Este documento explica las decisiones arquitectónicas más importantes del proyecto y su relación con el comportamiento del código actual verificado.

### 1) Administración del árbol de Branch-and-Bound A0

La solución exacta se encarga en `src/optimization/BranchAndBoundA0.cpp` y se apoya en `include/optimization/BranchAndBoundA0.h`. El diseño central consiste en mantener una cola `OPEN` ordenada por `best-bound` para explorar primero el nodo con menor cota inferior y, así, podar ramas más agresivamente.

La implementación actual hace tres cosas de forma explícita:

- reutiliza un único `ClpRelaxationSolver` para todos los nodos del árbol,
- aplica fijaciones por rama con `fixedToOne` y `fixedToZero`,
- usa `selectBranchVariable()` para elegir la variable fraccionaria más apropiada y expandir el árbol de forma controlada.

Esto tiene dos ventajas: reduce el costo de reconstruir modelos repetidamente y mantiene el algoritmo consistente con el enfoque de relajación lineal del enunciado. La rama exacta queda separada de `H1`, `H2` y `R`, de modo que cada estrategia pueda compararse bajo una misma interfaz de resultados y una misma validación del dominio.

### 2) Copia profunda de `primalColumnSolution()` y manejo del LP por nodo

La relajación LP se encapsula en `include/optimization/ClpRelaxationSolver.h`, `src/optimization/ClpRelaxationSolver.cpp` y `include/optimization/MatrixBuilder.h`. El punto crítico es que `CLP` no debe compartir memoria mutable con el resto de la lógica del proyecto.

Por eso se emplea una estrategia de dos niveles:

1. `MatrixBuilder` genera los datos del modelo, incluyendo `colLowerBounds`, `colUpperBounds`, `objective`, `rowLowerBounds`, `rowUpperBounds`, y un `CoinPackedMatrix`.
2. `ClpRelaxationSolver` toma ese modelo, lo carga en un `ClpSimplex`, y al resolver cada relajación hace una copia profunda del primal mediante `primalColumnSolution()`.

La copia profunda evita punteros colgantes y asegura que los cálculos del algoritmo `A0` puedan reutilizar el mismo solver, alterando solo cotas por nodo. Además, el Solver distingue entre estados `OPTIMAL`, `INFEASIBLE`, `LIMIT`, `ABANDONED` y `SOLVER_ERROR`, con clasificación explícita para evitar aceptar resultados ambiguos.

### 3) Patrón State para transiciones atómicas de celda y comandos del dominio

La representación del tablero y la evolución temporal se centra en `include/domain/Cell.h`, `include/domain/CellState.h`, `include/domain/Command.h`, `include/domain/Grid.h` y `src/domain/Grid.cpp`.

La decisión principal es que una celda no es un mero entero; es un objeto con un estado concreto (`OpenState`, `ClueState`, `SealedState`) que implementa el patrón State. Las transiciones válidas se gestionan en `CellState`, y cualquier cambio fuera del protocolo lanza una excepción (`std::runtime_error` o `std::logic_error`, según el caso de prueba).

Además:

- `Grid` aplica los comandos (`SELLAR`, `PISTA`, `LIBERAR`) sobre copias profundas para permitir un comportamiento atómico.
- cada `Snapshot` representa un estado inmutable de la época actual,
- `Validator` se usa como capa independiente para comprobar que una solución es consistente con las restricciones del problema.

Esto hace que la evolución del tablero sea verificable, resilient ante cambios de época y fácil de razonar en pruebas.

### 4) H2 como GRASP + búsqueda local + reglas deterministas de desempate

La heurística `H2` se implementa en `include/optimization/HeuristicaH2.h` y `src/optimization/HeuristicaH2.cpp`. El diseño está orientado a tres ideas:

- construir soluciones aleatorizadas con un `RCL` (`Restricted Candidate List`),
- usar una función de selección `h(i, j)` basada en coste normalizado y riesgo estimado,
- mejorar la solución generada mediante `LocalSearch` para intentar encontrar un mejor valor objetivo.

El proyecto además exige un comportamiento reproducible: la semilla efectiva se deriva a partir de `baseSeed + epoch`, y la construcción en GRASP toma decisiones aleatorias deterministas para la misma semilla. El código también usa desempates lexicográficos (`|F(i)|`, `-s_i`, `i`, `j`) para estabilizar los resultados frente a situaciones ambiguas.

El uso de `Simulator::runGraspMultiSeed()` confirma que la Sección 9 queda integrada en la arquitectura general, sin introducir un código aislado o ad-hoc.

### 5) Validador independiente y separación entre dominio, optimización y ejecución

La validación independiente del problema se encuentra en `include/domain/Validator.h` y `src/domain/Validator.cpp`. Esta decisión es fundamental en el proyecto porque permite separar tres responsabilidades distintas:

- el dominio responde al “qué es” del rompecabezas y sus invariantes,
- la optimización responde al “cómo resolverlo”,
- la ejecución (`main.cpp`, `Simulator`, `CSVExporter`) responde al “cómo se organiza la experimentación y la salida”.

El `Validator` es la capa que confirma si una solución:

- cumple la compatibilidad estructural `G_i == O_j`,
- respeta pistas y prohibiciones,
- satisface los requisitos de vecindad entre piezas,
- produce el costo objetivo esperado `Z^t(x)`.

Esto evita que la optimización confíe en el contenido implícito del `Grid` o `Snapshot` sin una verificación explícita. Además, el diseño permite que el mismo validador se use en pruebas, en `A0`, en `H1`, en `H2`, y en el `Simulator` sin duplicación funcional.

## Resumen ejecutivo

La arquitectura actual cumple tres objetivos de forma explícita:

1. separar responsabilidades por capas,
2. mantener un protocolo claro para transiciones de estado y evolución temporal,
3. permitir comparar distintos métodos de resolución bajo una misma validación y salida de resultados.

La combinación de `State + Command + Snapshot + Validator` con `Strategy + Branch-and-Bound + LP relaxation + GRASP` es precisamente lo que permite al proyecto ser extensible, verificable y consistente con el enunciado del RPE.
