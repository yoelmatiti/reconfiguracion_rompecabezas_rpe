# Compilador y flags
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -Wpedantic -O3 -DNDEBUG -Iinclude
LDFLAGS = -lCbc -lCgl -lOsiClp -lClp -lCoinUtils -lm

# Directorios
SRC_DIR = src
INC_DIR = include
TEST_DIR = tests
OBJ_DIR = obj
BIN_DIR = bin

# Archivos fuente compartidos del proyecto
SOURCES = $(wildcard $(SRC_DIR)/domain/*.cpp) \
          $(SRC_DIR)/RpeParser.cpp \
          $(wildcard $(SRC_DIR)/optimization/*.cpp)

MAIN_SOURCE = $(SRC_DIR)/main.cpp
TEST_SOURCES = $(wildcard $(TEST_DIR)/*.cpp)

# Objetos
OBJECTS = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SOURCES))
MAIN_OBJECT = $(OBJ_DIR)/main.o
TEST_OBJECTS = $(patsubst $(TEST_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(TEST_SOURCES))

# Targets principales
TARGET = $(BIN_DIR)/rpe_solver
TEST_TARGET = $(BIN_DIR)/test_runner
CBC_TEST_TARGET = $(BIN_DIR)/test_cbc_solver

# Reglas
all: directories $(TARGET)

test: directories $(TEST_TARGET) $(CBC_TEST_TARGET)
	./$(TEST_TARGET)
	./$(CBC_TEST_TARGET)

directories:
	mkdir -p $(OBJ_DIR)/domain $(OBJ_DIR)/parser $(OBJ_DIR)/optimization $(BIN_DIR) results

# Compilación del programa principal
$(TARGET): $(OBJECTS) $(MAIN_OBJECT)
	$(CXX) $(CXXFLAGS) $(OBJECTS) $(MAIN_OBJECT) -o $@ $(LDFLAGS)

# Compilación de los tests
$(TEST_TARGET): $(OBJECTS) $(TEST_OBJECTS)
	$(CXX) $(CXXFLAGS) $(OBJECTS) $(OBJ_DIR)/test_runner.o -o $@ $(LDFLAGS)

$(CBC_TEST_TARGET): $(OBJECTS) $(TEST_OBJECTS)
	$(CXX) $(CXXFLAGS) $(OBJECTS) $(OBJ_DIR)/test_cbc_solver.o -o $@ $(LDFLAGS)

# Regla genérica para objetos de src/
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Regla para objetos de tests
$(OBJ_DIR)/%.o: $(TEST_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Limpieza completa
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR) *.o

# Ejecución limpia del solver
run: $(TARGET)
	./$(TARGET)

.PHONY: all test clean run directories