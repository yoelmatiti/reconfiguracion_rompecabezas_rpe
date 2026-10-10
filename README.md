# RPE Solver

Este repositorio implementa una solución para el problema de reconfiguración del rompecabezas de encaje (RPE) usando una combinación de modelos LP/MIQP, patrones de diseño del dominio, heurísticas constructivas y un simulador temporal para evaluar múltiples métodos sobre cada época.

## Estado verificado

La base actual del proyecto ha sido verificada con:

- `make test`
- `./bin/test_cbc_solver`

Resultado verificado en esta sesión:

- `21/21` pruebas del suite `bin/test_runner` pasadas.
- `bin/test_cbc_solver` completó la prueba de CBC con estado `OPTIMAL`.

## Objetivo del proyecto

El programa procesa un archivo `.rpe` que describe:

- el tablero y las piezas,
- las pistas (`PISTA`),
- los eventos temporales (`SELLAR`, `PISTA`, `LIBERAR`),
- los parámetros de GRASP (`eta`, `lambda`, `seed`, `time_ms`).

A partir de ese archivo, el proyecto:

1. valida la instancia inicial,
2. construye un `Snapshot` del estado actual,
3. resuelve cada época con `A0`, `H1`, `H2` y `R`,
4. simula la evolución temporal mediante `Simulator`,
5. exporta resultados en CSV para análisis comparativo.

## Requisitos

- C++11
- GNU Make
- `g++`
- COIN-OR CBC, Clp, Cgl, OsiClp y CoinUtils
- Headers y librerías de desarrollo del sistema Linux

En Ubuntu/Debian, normalmente se requiere algo equivalente a:

```bash
sudo apt-get install coinor-libclp-dev coinor-libcbc-dev libosiclp-dev libcoinutils-dev
```

## Compilar

```bash
make
```

Esto genera el ejecutable principal en `bin/rpe_solver` y deja preparados los artefactos de tests.

## Ejecutar pruebas

```bash
make test
```

Esto compila y ejecuta:

- `bin/test_runner`
- `bin/test_cbc_solver`

## Ejecutar el programa principal

```bash
./bin/rpe_solver
```

### Menú interactivo

El programa ofrece el siguiente flujo principal:

1. Cargar caso `.rpe`
2. Mostrar y validar estado inicial
3. Ejecutar una época con `A0`, `H1`, `H2` y `R`
4. Ejecutar la secuencia completa (`Simulator`)
5. Exportar resultados CSV
6. Evaluación multi-semilla de GRASP (Sección 9)
7. Salir

## Modo CLI / batch

El ejecutable también admite una ejecución directa desde línea de comandos para una evaluación rápida de GRASP sobre una semilla específica:

```bash
./bin/rpe_solver --instance instances/basic_valid.rpe --eta 0.5 --lambda 0.0 --seed 42 --time_ms 2000
```

Salida esperada:

```text
RESULT,<ruta>,<eta>,<lambda>,<seed>,<bestCost>,<avgRuntimeMs>
```

Si no existe una solución factible en el tiempo dado, devuelve:

```text
RESULT,<ruta>,<eta>,<lambda>,<seed>,INFEASIBLE,<time_ms>
```

## Estructura de carpetas

```text
.
├── README.md
├── DEFENSA.md
├── Makefile
├── include/
│   ├── RpeParser.h
│   ├── domain/
│   │   ├── Cell.h
│   │   ├── CellState.h
│   │   ├── Command.h
│   │   ├── Grid.h
│   │   ├── GridGeometry.h
│   │   ├── Piece.h
│   │   ├── Side.h
│   │   ├── Snapshot.h
│   │   └── Validator.h
│   └── optimization/
│       ├── A0Strategy.h
│       ├── BranchAndBoundA0.h
│       ├── CbcReferenceSolver.h
│       ├── ClpRelaxationSolver.h
│       ├── CSVExporter.h
│       ├── H1Strategy.h
│       ├── H2Strategy.h
│       ├── HeuristicaH1.h
│       ├── HeuristicaH2.h
│       ├── LocalSearch.h
│       ├── LpRepairHeuristic.h
│       ├── MatrixBuilder.h
│       ├── PlacementStrategy.h
│       ├── RStrategy.h
│       ├── RunConfig.h
│       ├── Simulator.h
│       ├── SolveResult.h
│       └── SolverTypes.h
├── src/
│   ├── main.cpp
│   ├── RpeParser.cpp
│   ├── domain/
│   │   ├── Cell.cpp
│   │   ├── CellState.cpp
│   │   ├── Command.cpp
│   │   ├── Grid.cpp
│   │   ├── GridGeometry.cpp
│   │   ├── Piece.cpp
│   │   ├── Snapshot.cpp
│   │   └── Validator.cpp
│   └── optimization/
│       ├── A0Strategy.cpp
│       ├── BranchAndBoundA0.cpp
│       ├── CbcReferenceSolver.cpp
│       ├── ClpRelaxationSolver.cpp
│       ├── CSVExporter.cpp
│       ├── H1Strategy.cpp
│       ├── H2Strategy.cpp
│       ├── HeuristicaH1.cpp
│       ├── HeuristicaH2.cpp
│       ├── LocalSearch.cpp
│       ├── LpRepairHeuristic.cpp
│       ├── MatrixBuilder.cpp
│       ├── RStrategy.cpp
│       ├── Simulator.cpp
│       └── ...
├── tests/
│   ├── test_runner.cpp
│   └── test_cbc_solver.cpp
├── instances/
│   ├── basic_valid.rpe
│   ├── complex_test.rpe
│   ├── events_test.rpe
│   ├── invalid_test.rpe
│   ├── repair_factible.rpe
│   └── ...
├── results/
│   ├── out_results.csv
│   └── grasp_multiseed.csv
├── obj/
├── bin/
└── ...
```

## Casos incluidos

La carpeta `instances/` contiene varios ejemplos para pruebas y validación:

- `basic_valid.rpe`
- `complex_test.rpe`
- `events_test.rpe`
- `invalid_test.rpe`
- `repair_factible.rpe`

## Salidas y exportación CSV

### CSV principal

La aplicación puede exportar una tabla con la siguiente estructura:

```text
epoch,timestamp,method,status,objective,moved_pieces,runtime_ms,LB,UB,gap_percent,nodes,lp_calls,placement
```

Columna por columna:

- `epoch`: número de época procesada
- `timestamp`: marca temporal de la época
- `method`: `A0`, `H1`, `H2`, `R`, `SIMULATOR`
- `status`: resultado (`OPTIMAL`, `FEASIBLE`, `OK`, `INVALID`, etc.)
- `objective`: costo obtenido
- `moved_pieces`: piezas movidas o asignadas
- `runtime_ms`: tiempo de ejecución en milisegundos
- `LB`: cota inferior
- `UB`: cota superior
- `gap_percent`: brecha relativa
- `nodes`: nodos explorados por `A0`
- `lp_calls`: llamadas al LP
- `placement`: asignación `pieceId -> cellId`

### CSV multi-semilla de GRASP

La opción 6 genera además `results/grasp_multiseed.csv`, con métricas por semilla:

```text
seed,epoch,objective,runtimeMs,feasibleConstructions,bestFound
```

## Flujo recomendado para análisis o informe

1. Ejecutar `make clean && make`
2. Ejecutar `make test`
3. Cargar un caso `.rpe` desde el menú interactivo
4. Ejecutar la opción 3 para comparar `A0`, `H1`, `H2` y `R` sobre una época
5. Ejecutar la opción 4 para simular la secuencia completa
6. Usar la opción 5 para exportar `results/out_results.csv`
7. Para la Sección 9, usar la opción 6 para evaluar múltiples semillas

## Arquitectura resumida

El proyecto separa claramente tres capas:

- `domain`: `Grid`, `Cell`, `Piece`, `Snapshot`, `Validator`, `Command`
- `optimization`: `BranchAndBoundA0`, `ClpRelaxationSolver`, `MatrixBuilder`, `HeuristicH1`, `HeuristicH2`, `Simulator`
- `parser + execution`: `RpeParser`, `main.cpp`, `CSVExporter`

Esta división facilita:

- pruebas unitarias independientes,
- refactorización del código,
- comparación homogénea de los métodos,
- validación explícita del dominio antes de aceptar una solución.
