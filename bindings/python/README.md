# NEPath Bindings

Python bindings for NEPath C++ toolpath planning library using nanobind.

NEPath is a library for generating and optimizing toolpaths for additive manufacturing, including:
- Raster and Zigzag patterns
- Contour Parallel (CP) toolpaths
- Isoperimetric Quotient Optimization (IQOP)
- Tool compensation and offset operations
- Underfill and sharp corner analysis

## Installation

### Development Installation

```bash
cd bindings
pip install -e .
```

### Building from Source

```bash
cd bindings
pip install .
```

## Requirements

- Python >= 3.9
- CMake >= 3.15
- C++20 compatible compiler
- nanobind >= 1.3.2
- numpy >= 1.21.0

## Project Structure

```
.
├── cmake
│   └── NEPathConfig.py.in
├── CMakeLists.txt
├── examples
│   ├── demos.py
│   ├── plot_utils.py
│   ├── test_config.py
│   └── test_demos.py
├── NEPath
│   ├── __init__.py
│   ├── nepath_py.cpp
│   └── _nepath.pyi
├── pyproject.toml
└── README.md
```

```
bindings/
├── CMakeLists.txt          # CMake build configuration
├── pyproject.toml          # Python project configuration
├── src/
│   ├── nepath.h            # Precompiled header with common includes
│   ├── example.cpp         # Example C++ binding
│   └── nepath_bindings/
│       └── __init__.py     # Python package initialization
├── tests/
│   └── test_example.py     # Example tests
└── external/               # External dependencies (auto-downloaded)
    └── eigen/              # Eigen library (downloaded by CMake)
```

## Adding New Bindings

1. Create a new C++ file in `src/` (e.g., `mymodule.cpp`)
2. Include the precompiled header: `#include "nepath.h"`
3. Define your bindings using nanobind:
   ```cpp
   NB_MODULE(_mymodule, m) {
       m.doc() = "My module description";
       m.def("my_function", &my_function, "arg"_a, "My function description");
   }
   ```
4. Add the module to `CMakeLists.txt`:
   ```cmake
   add_nanobind_extension(_mymodule src/mymodule.cpp)
   ```
5. Import in `src/nepath_bindings/__init__.py`:
   ```python
   from ._mymodule import my_function
   ```

## Usage

### Quick Start

```python
import numpy as np
from nepath_bindings import NEPathPlanner, DirectParallelOptions

# Create a planner
planner = NEPathPlanner()

# Set a contour (example: circular shape)
theta = np.linspace(0, 2 * np.pi, 100)
x = 10 * np.cos(theta)
y = 10 * np.sin(theta)
planner.set_contour(x, y)

# Generate raster toolpaths
opts = DirectParallelOptions()
opts.delta = 1.0  # Line width
opts.angle = np.pi / 4  # 45 degree angle

paths = planner.Raster(opts)
print(f"Generated {len(paths)} toolpaths")

# Access path coordinates
for i, path in enumerate(paths):
    x_coords, y_coords = path.get_arrays()
    print(f"Path {i}: {len(x_coords)} waypoints")
```

### Running Demo Functions

The package includes demo functions that replicate the C++ demos:

```python
from nepath_bindings import demos

# Generate raster toolpaths
raster_paths = demos.demo_Raster()

# Generate zigzag toolpaths
zigzag_paths = demos.demo_Zigzag()

# Generate contour parallel toolpaths
cp_paths = demos.demo_CP()

# Generate CP with connected Fermat spiral
cp_cfs_paths = demos.demo_CP_CFS()

# Tool compensation
compensated_paths = demos.demo_tool_compensate()

# Underfill analysis
paths, underfill_solution = demos.demo_underfill()
print(f"Underfill rate: {underfill_solution.underfillrate * 100:.2f}%")

# Sharp corner detection
paths, sharp_corners = demos.demo_sharpcorner()
```

## Testing

```bash
cd bindings
pip install -e .
pytest tests/
```

## API Reference

### Main Classes

- **NEPathPlanner**: Main class for toolpath planning
  - `set_contour(x, y)`: Set the outer boundary
  - `addhole(x, y)`: Add an inner boundary (hole)
  - `Raster(opts)`: Generate raster toolpaths
  - `Zigzag(opts)`: Generate zigzag toolpaths
  - `CP(opts)`: Generate contour parallel toolpaths
  - `IQOP(opts)`: Generate optimized toolpaths (requires Gurobi)
  - `tool_compensate(opts)`: Offset contours

- **FileAgent**: Utilities for reading/writing CSV files
  - `write_csv(path, filename)`: Write a single path
  - `write_csv(paths, prefix, suffix)`: Write multiple paths
  - `read_csv(filename)`: Read a path from file

- **Curve**: Geometric operations and analysis
  - `UnderFill(contour, holes, paths, delta, reratio)`: Calculate underfill
  - `SharpTurn_Invariant(path, radius, threshold)`: Detect sharp corners
  - `wash_dis(path, distance)`: Resample path with uniform spacing

### Options

- **DirectParallelOptions**: Options for Raster/Zigzag
  - `delta`: Line width
  - `angle`: Angle in radians

- **ContourParallelOptions**: Options for CP toolpaths
  - `delta`: Line width
  - `wash`: Enable resampling
  - `washdis`: Resampling distance
  - `connect`: Connection algorithm (none, cfs, dfs)

- **NonEquidistantOptions**: Options for IQOP (requires Gurobi)
  - `delta`: Upper bound of line width
  - `alpha`: Lower bound scale
  - `optimize_Q`: Optimize isoperimetric quotient
  - `optimize_S`: Optimize area
  - `optimize_L`: Optimize length

## Resources

- [nanobind documentation](https://nanobind.readthedocs.io/)
- [scikit-build-core documentation](https://scikit-build-core.readthedocs.io/)
- [Eigen documentation](https://eigen.tuxfamily.org/)
