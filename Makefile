# Makefile for NEPath with IPOPT support
# Assumes IPOPT is installed via conda in compas_opzuid environment

# Compiler settings
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall
ENABLE_DIRECTION_PARALLEL = -DENABLE_DIRECTION_PARALLEL

# IPOPT paths (using Homebrew installation)
HOMEBREW_PREFIX = /opt/homebrew
IPOPT_INC = $(HOMEBREW_PREFIX)/include/coin-or
IPOPT_LIB = $(HOMEBREW_PREFIX)/lib

# Include and library paths
INCLUDES = -I. -INEPath-master -I$(IPOPT_INC)
LIBS = -L$(IPOPT_LIB) -lipopt -lm -ldl

# Source files
NEPATH_SOURCES = NEPath-master/Basic.cpp \
                 NEPath-master/Connector.cpp \
                 NEPath-master/ContourParallel.cpp \
                 NEPath-master/Curve.cpp \
                 NEPath-master/DirectionParallel.cpp \
                 NEPath-master/FileAgent.cpp \
                 NEPath-master/NEPathPlanner.cpp \
                 NEPath-master/PlanningOptions.cpp \
                 NEPath-master/clipper.cpp \
                 NEPath-master/path.cpp \
                 NEPath-master/NonEquidistant.cpp

DEMO_SOURCES = demos.cpp

# Object files
NEPATH_OBJECTS = $(NEPATH_SOURCES:.cpp=.o)
DEMO_OBJECTS = $(DEMO_SOURCES:.cpp=.o)

# Executables
TEST_DEMOS = test_demos

.PHONY: all clean test

all: $(TEST_DEMOS)

# Build test executable
$(TEST_DEMOS): main.cpp $(NEPATH_OBJECTS) $(DEMO_OBJECTS)
	$(CXX) $(CXXFLAGS) $(ENABLE_DIRECTION_PARALLEL) $(INCLUDES) -o $@ $^ $(LIBS)
	@echo "Built $@ successfully!"
	@echo "Run with: ./$(TEST_DEMOS)"

# Compile NEPath source files
NEPath-master/%.o: NEPath-master/%.cpp
	$(CXX) $(CXXFLAGS) $(ENABLE_DIRECTION_PARALLEL) $(INCLUDES) -c -o $@ $<

# Compile demo files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(ENABLE_DIRECTION_PARALLEL) $(INCLUDES) -c -o $@ $<

# Test target
test: $(TEST_DEMOS)
	@echo "Running demos..."
	./$(TEST_DEMOS)

# Clean build artifacts
clean:
	rm -f $(NEPATH_OBJECTS) $(DEMO_OBJECTS) $(TEST_DEMOS)
	rm -f NEPath-master/*.o *.o
	@echo "Cleaned build artifacts"

# Help target
help:
	@echo "NEPath Makefile with IPOPT support"
	@echo ""
	@echo "Targets:"
	@echo "  all        - Build test_demos executable (default)"
	@echo "  test       - Build and run demos"
	@echo "  clean      - Remove build artifacts"
	@echo "  help       - Show this help message"
	@echo ""
	@echo "Requirements:"
	@echo "  - IPOPT installed via conda"
	@echo "  - Activate conda environment: conda activate compas_opzuid"
	@echo ""
	@echo "Usage:"
	@echo "  make           # Build"
	@echo "  make test      # Build and run"
	@echo "  make clean     # Clean"
