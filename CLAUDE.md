# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

NEPath is a toolpath planning library for additive manufacturing (3D printing) and CNC milling, written in C++17 with Python bindings. The library generates 1D toolpaths to fill 2D slices using various algorithms:

- **Optimization-based non-equidistant toolpaths**: IQOP (Isoperimetric-Quotient-Optimal Toolpath) and variants
- **Classical toolpaths**: Contour-Parallel (CP), Zigzag, Raster
- **Toolpath connection algorithms**: Connected Fermat Spiral (CFS), Depth First Search (DFS)
- **Analysis tools**: Tool compensation, underfill calculation, sharp corner detection

The project consists of two main components:
1. **NEPath-master/**: Core C++ library
2. **bindings/**: Python bindings using nanobind

## Build and Development Commands

### C++ Library (Visual Studio)

The C++ library uses Visual Studio projects (`.vcxproj`, `.sln`) for building on Windows.

**Note**: The library depends on [Gurobi](https://www.gurobi.com/) optimizer for IQOP and optimization-based toolpaths. To build without Gurobi, comment out `#define IncludeGurobi` in `NEPath-master/setup_NEPath.h`.

### Python Bindings

The Python bindings use CMake and scikit-build-core for building.

**Install in development mode** (recommended):
```bash
cd bindings
pip install -e .
```

**Build from source**:
```bash
cd bindings
pip install .
```

**Run tests**:
```bash
cd bindings
pytest tests/
```

**Clean build artifacts**:
```bash
cd bindings
rm -rf build/ dist/ *.egg-info
```

**Code style** (Black/Ruff):
```bash
cd bindings
ruff check .
ruff format .
```

### Dependencies

C++ library dependencies:
- C++17 compiler
- [Clipper1](https://sourceforge.net/projects/polyclipping/) (polygon clipping, included)
- [Gurobi](https://www.gurobi.com/) (optional, for IQOP)

Python bindings dependencies:
- Python >= 3.9
- CMake >= 3.15
- C++20 compatible compiler
- nanobind >= 1.3.2
- numpy >= 1.21.0
- Eigen >= 3.4.0 (header-only, auto-downloaded by CMake)

## Architecture

### Core C++ Library Structure

The NEPath library is organized around the `NEPathPlanner` class, which is the primary interface for all toolpath generation operations.

**Key Classes and Modules**:

1. **NEPathPlanner** (`NEPathPlanner.h/cpp`): Main interface class
   - Manages slice contours and holes
   - Dispatches to specific toolpath generators
   - Handles tool compensation (offsetting)

2. **path/paths** (`path.h/cpp`): Core data structures
   - `path`: Stores a single toolpath as x/y coordinate arrays
   - `paths`: Vector of path objects
   - `pathnode`: Tree structure for toolpath connections

3. **Toolpath Generators**:
   - `ContourParallel.h/cpp`: Generates CP (contour-parallel) toolpaths by successive offsetting
   - `DirectionParallel.h/cpp`: Generates Raster and Zigzag toolpaths
   - `NonEquidistant.h/cpp`: Generates IQOP toolpaths (requires Gurobi)

4. **Curve** (`Curve.h/cpp`): Geometric utilities
   - Underfill calculation
   - Sharp corner detection using area invariant
   - Path resampling and washing

5. **Connector** (`Connector.h/cpp`): Toolpath connection algorithms
   - Connected Fermat Spiral (CFS)
   - Depth First Search (DFS)

6. **FileAgent** (`FileAgent.h/cpp`): CSV file I/O for toolpaths

7. **PlanningOptions** (`PlanningOptions.h/cpp`): Option structures
   - `DirectParallelOptions`: For Raster/Zigzag
   - `ContourParallelOptions`: For CP
   - `NonEquidistantOptions`: For IQOP
   - `ConnectAlgorithm`: Enumeration (none, cfs, dfs)

**Conditional Compilation**:
- `IncludeGurobi`: Defined in `setup_NEPath.h`, enables/disables Gurobi-dependent code
- `ENABLE_DIRECTION_PARALLEL`: Enables Raster/Zigzag support

### Python Bindings Architecture

The Python bindings are structured as a single extension module that wraps the C++ library:

**Build System**:
- CMake builds the C++ extension modules
- scikit-build-core integrates CMake with Python packaging
- nanobind generates Python-C++ bindings
- `CMakeLists.txt` defines the `_nepath` extension module
- Precompiled headers (`nepath.h`) speed up compilation

**Key Files**:
- `bindings/src/nepath_wrapper.cpp`: Main binding code that exposes C++ classes to Python
- `bindings/src/nepath_bindings/__init__.py`: Python package entry point
- `bindings/CMakeLists.txt`: Build configuration, includes NEPath sources and sets compiler flags

**Important CMake Configuration**:
- Gurobi is disabled by default: `IncludeGurobi=0`
- Direction parallel is enabled: `ENABLE_DIRECTION_PARALLEL`
- Eigen is auto-downloaded if not present
- Warnings are suppressed for deprecated 'register' keyword (legacy Clipper code)

### Data Flow

1. **Input**: User defines a slice contour and optional holes using x/y coordinate arrays
2. **Preprocessing**: Contours are optionally "washed" (resampled) for uniform spacing
3. **Generation**: NEPathPlanner dispatches to appropriate generator based on method
4. **Offsetting**: Generators use Clipper library for polygon offsetting operations
5. **Connection** (optional): Separate toolpaths are connected using CFS or DFS
6. **Output**: Returns `paths` vector containing generated toolpaths

### Tool Compensation Pattern

Many toolpath operations require "tool compensation" to account for tool width:
- For additive manufacturing, the outer boundary should be offset **inward** by half the line width before planning
- Use `tool_compensate()` with **negative delta** to offset inward
- Use positive delta to offset outward
- This pattern is used extensively in CP and IQOP examples

## Important Implementation Details

### Path Memory Management

The `path` struct manually manages memory with `double* x` and `double* y`:
- Use `copy_with_new()` to deep copy
- Use `steal()` to transfer ownership
- Use `clear_with_delete()` to free memory
- Constructors/destructors handle allocation/deallocation

### Resampling ("Washing")

Most methods accept wash parameters:
- `wash`: Enable uniform resampling
- `washdis`: Maximum distance between waypoints
- `num_least`: Minimum number of waypoints
- Recommended to keep `wash=true` for consistent toolpath quality

### Angle Convention

For Raster/Zigzag toolpaths:
- `angle` is in **radians** relative to the x-axis
- Use `acos(-1.0)` to get π in C++
- Use `np.pi` in Python

### Underfill Computation

Underfill represents unfilled regions in a slice:
- Uses discrete grid sampling (resolution controlled by `reratio`)
- Returns `UnderFillSolution` with 2D boolean maps
- `underfillrate` is the percentage of unfilled area

### Sharp Corner Detection

Uses area invariant method (Pottmann et al., 2009):
- Rolling circle with specified radius
- Threshold on area percentage to determine sharp corners
- Returns `SharpTurnSolution` with boolean array marking sharp points

## Testing

The C++ library includes demo functions in `demos.cpp` that generate example toolpaths and write them to `data_examples/`. These serve as integration tests and usage examples.

The Python bindings replicate these demos as test cases in `bindings/tests/test_demos.py`.

## Citation

If using this library, cite the IQOP paper:
```
Wang Y, Hu C, Wang Z, et al. Optimization-based non-equidistant toolpath planning for robotic additive manufacturing with non-underfill orientation. Robotics and Computer-Integrated Manufacturing, 2023, 84: 102599.
```
