# NEPath: Python Tutorial

## Setup

###  Environment Variables

Please set the following environment variables (global or ipopt at the cmake-terminal)

| Required when enabling: | Key              | Value (Example, Linux)                  |
| --------------------- | ---------------- | --------------------------------------- |
| Gurobi-based IQOP     | `GUROBI_HOME`    | `/home/usr/software/gurobi1300/linux64` |
| Gurobi-based IQOP     | `GUROBI_VERSION` | `130`                                   |
| Ipopt-based IQOP      | `IPOPT_ROOT`     | `/home/usr/miniconda3/envs/env-NEPath`  |

+ The environment variable `IPOPT_ROOT` does not need to be set if IPOPT can be located via `pkg-config`.
+ For Ipopt, you can check the environment variable `IPOPT_ROOT` if the folder `$IPOPT_ROOT/include/coin` exists.
+ For gurobi, you can check the environment variable `GUROBI_HOME` if the file `$GUROBI_HOME/include/gurobi_c++.h` exists. You can check the environment variable `GUROBI_VERSION` if `$GUROBI_HOME/lib/gurobi$GUROBI_VERSION.*` exists. For example, if you use gurobi v13.0.0, please set `GUROBI_VERSION` by `130`.

### Requirements

- Python >= 3.9
- CMake >= 3.15
- C++20 compatible compiler
- nanobind >= 1.3.2
- numpy >= 1.21.0

### Project Structure

```shell
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

## Install

### Choice 1. PIP Install NEPath (from Pypi)

TODO

### Choice 2. PIP Install NEPath (from source code)


Open the Terminal and run:

```shell
git clone https://github.com/WangY18/NEPath.git
cd NEPath
cd bindings
cd python
```

#### Choice 2.a. Install NEPath in Linux / macOS

```shell
export NEPATH_ENABLE_IPOPT=OFF # (Optional: disable Ipopt)
export NEPATH_ENABLE_GUROBI=OFF # (Optional: disable gurobi) 
pip install .
```

#### Choice 2.b. Install NEPath in Windows

In Powershell:

```shell
$env:NEPATH_ENABLE_IPOPT="OFF" # (Optional: disable Ipopt)
$env:NEPATH_ENABLE_GUROBI="OFF" # (Optional: disable gurobi) 
pip install .
```

Or in cmd:

```shell
setx NEPATH_ENABLE_IPOPT OFF # (Optional: disable Ipopt)
setx NEPATH_ENABLE_GUROBI OFF # (Optional: disable gurobi) 
pip install .
```

#### Uninstall

```shell
pip uninstall nepath-py
```

### Choice 3. CMAKE Install NEPath (from source code)

Open the Terminal and run:

```shell
git clone https://github.com/WangY18/NEPath.git
cd NEPath
cd bindings
cd python
mkdir build
cd build
```

#### Choice 3.a. Install NEPath in Linux / macOS

Open the Terminal and run:

```shell
cmake ..
cmake --build .
cmake --install .
```

If you want to disable Gurobi and Ipopt support, replace the above code `cmake ..` by

```shell
cmake .. -DNEPATH_ENABLE_GUROBI=OFF -DNEPATH_ENABLE_IPOPT=OFF
```

If you want to install the NEPath library at another directory `example_path`, replace the above code `cmake --install .` by

```shell
make --install . --prefix example_path
# e.g. make --install . --prefix $CONDA_PREFIX
```


You can uninstall the NEPath library like this

```shell
# replace by your dir
rm -f /home/robot/miniconda3/envs/yunan-NEPath/lib/python3.13/site-packages/_nepath.cpython-313-x86_64-linux-gnu.so
rm -rf /home/robot/miniconda3/envs/yunan-NEPath/lib/python3.13/site-packages/NEPath
```

#### Choice 3.b Install NEPath in Windows with MSVC

Open PowerShell or CMD and run:

```shell
cmake .. -G "Visual Studio 17 2022" -A x64  # Replace with your VS version
cmake --build . --config Release
cmake --install . --config Release
```

If you want to disable Gurobi and Ipopt support, replace the above code `cmake ..` by

```shell
cmake .. -G "Visual Studio 17 2022" -A x64 -DNEPATH_ENABLE_GUROBI=OFF -DNEPATH_ENABLE_IPOPT=OFF
```

If you want to install the NEPath library at another directory like `C:/Path/To/Install/NEPath`, replace the above code `make --install . --config Release` by

```shell
cmake --install . --config Release --prefix "C:/Path/To/Install/NEPath"
```

You can uninstall the NEPath library like this

```shell
# replace by your dir
del "C:/ProgramData/anaconda3/envs/NEPath/Lib/site-packages/_nepath.cp311-win_amd64.pyd"
Remove-Item "C:\ProgramData\anaconda3\envs\NEPath\Lib\site-packages\NEPath" -Recurse -Force
```

#### Choice 3.c Install NEPath in Windows with GNU

Open your MinGW/MSYS2 shell or WSL terminal, and run

```shell
cmake .. -G "MinGW Makefiles" # or: cmake .. -G "Ninja"
```

If you want to disable Gurobi and Ipopt support, replace the above code `cmake ..` by

```shell
cmake .. -G "MinGW Makefiles" -DNEPATH_ENABLE_GUROBI=OFF -DNEPATH_ENABLE_IPOPT=OFF
```

If you want to install the NEPath library at another directory like `C:/Path/To/Install/NEPath`, replace the above code `cmake --install . --config Release` by

```shell
cmake --install . --prefix "C:/Path/To/Install/NEPath"
```

## Testing

```shell
# Assume that you are now in the NEPath/bindings/python/examples path
mkdir data_examples
mkdir plot_outputs
python test_demos.py
```

## API

Please refer to `_nepath.h`

### Main Classes

- **NEPathPlanner**: Main class for toolpath planning
    - `set_contour(x, y)`: Set the outer boundary
    - `addhole(x, y)`: Add an inner boundary (hole)
    - `Raster(opts)`: Generate raster toolpaths
    - `Zigzag(opts)`: Generate zigzag toolpaths
    - `CP(opts)`: Generate contour parallel toolpaths
    - `IQOP(opts)`: Generate optimized toolpaths (requires Ipopt / Gurobi)
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
    - `optimize_Q`(bool): Optimize isoperimetric quotient
    - `optimize_S`(bool): Optimize area
    - `optimize_L`(bool): Optimize length
    - `lambda_Q`: Weighting coefficient for isoperimetric quotient
    - `lambda_S`: Weighting coefficient for area
    - `lambda_L`: Weighting coefficient for length
    - `connector`(`ConnectAlgorithm`): Enum of connection algorithms, including `none`, `cfs`, `dfs`.
    - `optimizer`(`OptimizationAlgorithm`): Connection algorithm, including `none`, `ipopt`, `gurobi`.


## Examples

### Toolpath Generation

#### IQOP (Isoperimetric-Quotient-Optimal Toolpath, Wang Y et al., 2023)

##### Based on Gurobi


```python
import numpy as np
import NEPath as nepath

if __name__ == "__main__":
    # Create contour (slightly different for IQOP)
    theta = np.linspace(0, 2 * np.pi, 1000)
    r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Set parameters
    delta = 1.0
    alpha = 0.5
    washdis = 0.2

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    # Initialize NEPath planner and set contour
    planner = nepath.NEPathPlanner()
    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = nepath.NonEquidistantOptions()
    opts.delta = delta
    opts.alpha = alpha
    opts.dot_delta = 1.0
    opts.ddot_delta = 0.1
    opts.optimize_Q = True
    opts.optimize_S = False
    opts.optimize_L = False
    opts.lambda_Q = 1.0
    opts.wash = True
    opts.washdis = washdis
    opts.optimizer = nepath.OptimizationAlgorithm.gurobi  # Use gurobi solver

    # Generate IQOP paths
    IQOP_paths = planner.IQOP(opts, False)  # log=False for cleaner output
    print(f"There are {len(IQOP_paths)} continuous toolpaths in total.")
```

<p align="center">
	<img src="https://github.com/WangY18/NEPath/assets/75420225/6f5a6369-1eda-444f-838b-c260aac58ecf" alt="IQOP-gurobi" height="300" />
</p>
<p align="center">
	<b>Figure.</b> IQOP toolpath minimizing Q based on gurobi.
</p>

<p align="center">
	<img src="https://github.com/WangY18/NEPath/assets/75420225/65b5b1a7-ebc4-4f8b-b2d0-f72352afda9d" alt="IQSOP-gurobi" height="300" />
</p>
<p align="center">
	<b>Figure.</b> IQOP toolpath minimizing Q+1.0S based on gurobi.
</p>

<p align="center">
	<img src="https://github.com/WangY18/NEPath/assets/75420225/aac675c2-644e-4ff2-b608-36d26226fc26" alt="IQLOP-gurobi" height="300" />
</p>
<p align="center">
	<b>Figure.</b> IQOP toolpath minimizing L based on gurobi.
</p>

##### Based on Ipopt


```python
import numpy as np
import NEPath as nepath

if __name__ == "__main__":
    # Create contour (slightly different for IQOP)
    theta = np.linspace(0, 2 * np.pi, 1000)
    r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Set parameters
    delta = 1.0
    alpha = 0.5
    washdis = 0.2

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    # Initialize NEPath planner and set contour
    planner = nepath.NEPathPlanner()
    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = nepath.NonEquidistantOptions()
    opts.delta = delta
    opts.alpha = alpha
    opts.dot_delta = 1.0
    opts.ddot_delta = 0.1
    opts.optimize_Q = True
    opts.optimize_S = False
    opts.optimize_L = False
    opts.lambda_Q = 1.0
    opts.wash = True
    opts.washdis = washdis
    opts.connector = nepath.ConnectAlgorithm.none  # none / cfs / dfs
    opts.optimizer = nepath.OptimizationAlgorithm.ipopt  # Use ipopt solver

    # Generate IQOP paths
    IQOP_paths = planner.IQOP(opts, False)  # log=False for cleaner output
    print(f"There are {len(IQOP_paths)} continuous toolpaths in total.")
```

<p align="center">
	<img src="https://github.com/user-attachments/assets/a7c03420-5131-446a-8f8b-6a6174281012" alt="IQOP-ipopt" height="300" />
</p>
<p align="center">
	<b>Figure.</b> IQOP toolpath minimizing L based on ipopt.
</p>

#### CP (Contour-Parallel)

```python
import numpy as np
import NEPath as nepath

if __name__ == "__main__":
    # Create contour (slightly different for IQOP)
    theta = np.linspace(0, 2 * np.pi, 1000)
    r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Set parameters
    delta = 1.0
    washdis = 0.2

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    # Initialize NEPath planner and set contour
    planner = nepath.NEPathPlanner()
    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = nepath.ContourParallelOptions()
    opts.delta = delta
    opts.wash = True
    opts.washdis = washdis

    # Generate CP paths
    CP_paths = planner.CP(opts)
    print(f"There are {len(CP_paths)} continuous toolpaths in total.")
    for i, path in enumerate(CP_paths):
        print(f"Toolpath {i} has {path.length} waypoints.")
```

<p align="center">
	<img src="https://user-images.githubusercontent.com/75420225/229331109-58155b93-d897-4553-b923-28be4eecfee1.png" alt="CP" height="300" />
</p>
<p align="center">
	<b>Figure.</b> CP toolpath.
</p>


#### Zigzag

```python
import numpy as np
import NEPath as nepath

if __name__ == "__main__":
    # Create contour (slightly different for IQOP)
    theta = np.linspace(0, 2 * np.pi, 1000)
    r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Set parameters
    delta = 1.0
    angle = np.pi / 3.0

    # Initialize NEPath planner and set contour
    planner = nepath.NEPathPlanner()
    planner.set_contour(x, y)

    # Set toolpath parameters
    opts = nepath.DirectParallelOptions()
    opts.delta = delta
    opts.angle = angle

    # Generate zigzag paths
    zigzag_paths = planner.Zigzag(opts)
    print(f"There are {len(zigzag_paths)} continuous toolpaths in total.")
    for i, path in enumerate(zigzag_paths):
        print(f"Toolpath {i} has {path.length} waypoints.")
```

<p align="center">
	<img src="https://user-images.githubusercontent.com/75420225/229331070-887c0172-7ad4-42dd-85a1-6c35f09a6cc4.png" alt="zigzag" height="300" />
</p>
<p align="center">
	<b>Figure.</b> Zigzag toolpath.
</p>


#### Raster

```python
import numpy as np
import NEPath as nepath

if __name__ == "__main__":
    # Create contour (slightly different for IQOP)
    theta = np.linspace(0, 2 * np.pi, 1000)
    r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Set parameters
    delta = 1.0
    angle = np.pi / 3.0

    # Initialize NEPath planner and set contour
    planner = nepath.NEPathPlanner()
    planner.set_contour(x, y)

    # Set toolpath parameters
    opts = nepath.DirectParallelOptions()
    opts.delta = delta
    opts.angle = angle

    # Generate raster paths
    raster_paths = planner.Raster(opts)
    print(f"There are {len(raster_paths)} continuous toolpaths in total.")
    for i, path in enumerate(raster_paths):
        print(f"Toolpath {i} has {path.length} waypoints.")
```

<p align="center">
	<img src="https://user-images.githubusercontent.com/75420225/229333683-670c0890-1a34-4e2a-b1ca-5de918815e09.png" alt="raster" height="300" />
</p>
<p align="center">
	<b>Figure.</b> Raster toolpath.
</p>


### Toolpath Connection

#### IQOP connected by CFS

CP can be connected by CFS in the same way.

```python
import numpy as np
import NEPath as nepath

if __name__ == "__main__":
    # Create contour (slightly different for IQOP)
    theta = np.linspace(0, 2 * np.pi, 1000)
    r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Set parameters
    delta = 1.0
    alpha = 0.5
    washdis = 0.2

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    # Initialize NEPath planner and set contour
    planner = nepath.NEPathPlanner()
    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = nepath.NonEquidistantOptions()
    opts.delta = delta
    opts.alpha = alpha
    opts.dot_delta = 1.0
    opts.ddot_delta = 0.1
    opts.optimize_Q = True
    opts.optimize_S = False
    opts.optimize_L = False
    opts.lambda_Q = 1.0
    opts.wash = True
    opts.washdis = washdis
    opts.connector = nepath.ConnectAlgorithm.cfs  # none / cfs / dfs
    opts.optimizer = nepath.OptimizationAlgorithm.gurobi  # Use gurobi solver

    # Generate IQOP paths
    IQOP_paths = planner.IQOP(opts, False)  # log=False for cleaner output
    print(f"There are {len(IQOP_paths)} continuous toolpaths in total.")
```

<p align="center">
	<img src="https://github.com/user-attachments/assets/38b01606-ff3e-4149-89c8-39335ed70ac3" alt="iqop_cfs" height="300" />
</p>
<p align="center">
	<b>Figure.</b> IQOP connected by CFS.
</p>

#### IQOP connected by DFS

CP can be connected by DFS in the same way.

```python
import numpy as np
import NEPath as nepath

if __name__ == "__main__":
    # Create contour (slightly different for IQOP)
    theta = np.linspace(0, 2 * np.pi, 1000)
    r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Set parameters
    delta = 1.0
    alpha = 0.5
    washdis = 0.2

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    # Initialize NEPath planner and set contour
    planner = nepath.NEPathPlanner()
    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = nepath.NonEquidistantOptions()
    opts.delta = delta
    opts.alpha = alpha
    opts.dot_delta = 1.0
    opts.ddot_delta = 0.1
    opts.optimize_Q = True
    opts.optimize_S = False
    opts.optimize_L = False
    opts.lambda_Q = 1.0
    opts.wash = True
    opts.washdis = washdis
    opts.connector = nepath.ConnectAlgorithm.dfs  # none / cfs / dfs
    opts.optimizer = nepath.OptimizationAlgorithm.gurobi  # Use gurobi solver

    # Generate IQOP paths
    IQOP_paths = planner.IQOP(opts, False)  # log=False for cleaner output
    print(f"There are {len(IQOP_paths)} continuous toolpaths in total.")
```

<p align="center">
	<img src="https://github.com/user-attachments/assets/c03acb4f-c14f-40b9-81f5-ad49f758611d" alt="iqop_dfs" height="300" />
</p>
<p align="center">
	<b>Figure.</b> IQOP connected by DFS.
</p>
### Others

#### Tool  compensate

```python
import numpy as np
import NEPath as nepath

if __name__ == "__main__":
    # Create contour (slightly different for IQOP)
    theta = np.linspace(0, 2 * np.pi, 1000)
    r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Add a hole
    x_hole = np.array([-5, 5, 5, 0, -5], dtype=float)
    y_hole = np.array([-5, -5, 5, 0, 5], dtype=float)

    # Initialize NEPath planner and set contour
    planner = nepath.NEPathPlanner()
    planner.set_contour(x, y)
    planner.addhole(x_hole, y_hole)

    # Set toolpath parameters
    delta_offset = -1.5
    washdis = 0.2

    # Tool compensate
    opts = nepath.ContourParallelOptions()
    opts.delta = delta_offset
    opts.wash = True
    opts.washdis = washdis
    ps_toolcompensate = planner.tool_compensate(opts)

    print(f"There are {len(ps_toolcompensate)} continuous toolpaths in total.")
    for i, path in enumerate(ps_toolcompensate):
        print(f"Toolpath {i} has {path.length} waypoints.")
```

<p align="center">
	<img src="https://user-images.githubusercontent.com/75420225/229334415-da87d12b-8c18-4a2b-b646-a8fe7558b06e.png" alt="Tool compensate" height="300" />
</p>
<p align="center">
	<b>Figure.</b> Tool compensate.
</p>


#### Underfill

```python
import numpy as np
import NEPath as nepath

if __name__ == "__main__":
    # Create contour (slightly different for IQOP)
    theta = np.linspace(0, 2 * np.pi, 1000)
    r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Set parameters
    delta = 1.0
    washdis = 0.2
    reratio = 0.03

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    # Initialize NEPath planner and set contour
    planner = nepath.NEPathPlanner()
    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = nepath.ContourParallelOptions()
    opts.delta = delta
    opts.wash = True
    opts.washdis = washdis

    # Generate CP paths
    CP_paths = planner.CP(opts)
    print(f"There are {len(CP_paths)} continuous toolpaths in total.")

    # Calculate underfill
    contour_path = nepath.Path.from_arrays(x, y)
    ufs = nepath.Curve.UnderFill(contour_path, [], CP_paths, delta, reratio)
    print(f"The underfill rate is {ufs.underfillrate * 100:.2f}%.")
```

<p align="center">
	<img src="https://github.com/WangY18/NEPath/assets/75420225/e050a611-8108-4f4d-b293-b801443de746" alt="Underfill" height="300" />
</p>
<p align="center">
    <b>Figure.</b> Underfill. The underfill rate is 1.2401% in this example.
</p>


#### Sharp corner

```python
import numpy as np
import NEPath as nepath

if __name__ == "__main__":
    # Create contour (slightly different for IQOP)
    theta = np.linspace(0, 2 * np.pi, 1000)
    r = 15.0 * (1.0 + 0.1 * np.cos(10.0 * theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Set parameters
    delta = 1.0
    washdis = 0.2
    radius = 1.0
    threshold = 0.3

    # Tool compensate to get the outmost toolpath
    planner_toolcompensate = nepath.NEPathPlanner()
    planner_toolcompensate.set_contour(x, y)
    opts_toolcompensate = nepath.ContourParallelOptions()
    opts_toolcompensate.delta = -0.5 * delta
    opts_toolcompensate.wash = True
    opts_toolcompensate.washdis = washdis
    path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate)

    # Initialize NEPath planner and set contour
    planner = nepath.NEPathPlanner()
    planner.set_contour(path_outmost[0])

    # Set toolpath parameters
    opts = nepath.ContourParallelOptions()
    opts.delta = delta
    opts.wash = True
    opts.washdis = washdis

    # Generate CP paths
    CP_paths = planner.CP(opts)
    print(f"There are {len(CP_paths)} continuous toolpaths in total.")

    # Detect sharp corners
    num = 0
    ps_sharpturn = []
    for path in CP_paths:
        sol = nepath.Curve.SharpTurn_Invariant(path, radius, threshold, True, 0.5)
        sharp_turn = sol.get_sharp_turn()
        num += sum(sharp_turn)

        # Store results (SharpTurn values as x, AreaPercent as y)
        area_percent = sol.get_area_percent()
        sharp_path = nepath.Path()
        sharp_path.set_arrays(np.array([float(st) for st in sharp_turn], dtype=float), np.array(area_percent, dtype=float))
        ps_sharpturn.append(sharp_path)

    print(f"There exist {num} sharp corners.")
```

<p align="center">
	<img src="https://github.com/WangY18/NEPath/assets/75420225/4586213c-d5e2-415b-b81a-d09ed680ffe9" alt="Underfill" height="300" />
</p>
<p align="center">
    <b>Figure.</b> Sharp corners. There exist 44 sharp corners in this example.
</p>
### Others

#### Test Config

```python
from NEPath import IncludeIpopt, IncludeGurobi

if __name__ == "__main__":
    print(f"IncludeIpopt: {IncludeIpopt}")
    print(f"IncludeGurobi: {IncludeGurobi}")
```

