# NEPath: Python Tutorial

## Setup and Install

### Step 1. Environment Variables

Please set the following environment variables (global or ipopt at the cmake-terminal)

| Required when enabling: | Key              | Value (Example, Linux)                  |
| --------------------- | ---------------- | --------------------------------------- |
| Gurobi-based IQOP     | `GUROBI_HOME`    | `/home/usr/software/gurobi1300/linux64` |
| Gurobi-based IQOP     | `GUROBI_VERSION` | `130`                                   |
| Ipopt-based IQOP      | `IPOPT_ROOT`     | `/home/usr/miniconda3/envs/env-NEPath`  |

+ The environment variable `IPOPT_ROOT` does not need to be set if IPOPT can be located via `pkg-config`.
+ For Ipopt, you can check the environment variable `IPOPT_ROOT` if the folder `$IPOPT_ROOT/include/coin` exists.
+ For gurobi, you can check the environment variable `GUROBI_HOME` if the file `$GUROBI_HOME/include/gurobi_c++.h` exists. You can check the environment variable `GUROBI_VERSION` if `$GUROBI_HOME/lib/gurobi$GUROBI_VERSION.*` exists. For example, if you use gurobi v13.0.0, please set `GUROBI_VERSION` by `130`.

### Step 2. Install NEPath (from source code)

Open the Terminal and run:

```shell
git clone https://github.com/WangY18/NEPath.git
cd NEPath./bindings/python
mkdir build
cd build
```

#### Step 2.a. Install NEPath in Linux / macOS

Open the Terminal and run:

```shell
cmake ..
cmake --build .
cmake --install .
```

If you want to disable Gurobi and Ipopt support, replace the above code `cmake ..` by

```shell
sudo cmake .. -DNEPATH_ENABLE_GUROBI=OFF -DNEPATH_ENABLE_IPOPT=OFF
```

If you want to install the NEPath library at another directory `example_path`, replace the above code `cmake --install .` by

```shell
make --install . --prefix example_path
# e.g. make --install . --prefix $CONDA_PREFIX
```

#### Step 2.b.a Install NEPath in Windows with MSVC

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


#### Step 2.b.b Install NEPath in Windows with GNU

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

## API

#### `NEPath/src/path.h`

+ `(struct)path` is a struct to store information of toolpaths. `(double*)path::x` and `(double*)path::y` are waypoints of a toolpath.
+ `paths` is a `vector` of `path`, i.e., `typedef vector<path> paths;`

#### `NEPath/src/NEPathPlanner.h`

The package `NEPathPlanner.h` include the key class of **NEPath**, i.e., `NEPathPlanner`. All operations on toolpath planning is based on `NEPathPlanner`. The API of `NEPathPlanner` is as follows:

+ `(void)NEPathPlanner::set_contour()`: Set the contour, i.e., the outer boundary, of the slice. Every slice only have one closed contour. The start point and the end point of the contour are connected by default. If you want to set the outmost toolpath has a distance from the actual outer boundary, you can call `NEPathPlanner::tool_compensate()` with a **negative** distance to obtain the outmost toolpath firstly, and set the obtained outmost toolpath as the boundary of a new slice. See the example of Zigzag and CP for details.
	+ `(const double*)x`, `(const double*)y`, `(int)length`: The number of waypoints is  `length`. The `i`-th waypoint is `(x[i],y[i])`. It can be substituted as `(const path&)contour_new`.
	+ `(bool)wash`, `(double)wash_dis`, `(int)num_least`: If `wash==true`, the contour would be resampled with a uniformly-distributed distance no more than `wash_dis`, and the number of waypoints are no less than `num_least`. By default, `wash=true, washdis=0.2, num_least=50  `.
	+ The contour is stored in a public member variable `(path)contour`.
+ `(void)NEPathPlanner::addhole()`: Add a new hole, i.e., the inner boundaries, onto the slice. The start point and the end point of every hole are connected by default. A slice is allowed to have no holes. The same as `(void)NEPathPlanner::set_contour()`, you can call `NEPathPlanner::tool_compensate()` to offset the added hole.
	+ `(const double*)x`, `(const double*)y`, `(int)length`: The number of waypoints is  `length`. The `i`-th waypoint is `(x[i],y[i])`. It can be substituted as `(const path&)hole_new`.
	+ `(bool)wash`, `(double)wash_dis`, `(int)num_least`: If `wash==true`, the contour would be resampled with a uniformly-distributed distance no more than `wash_dis`, and the number of waypoints are no less than `num_least`. By default, `wash=true, washdis=0.2, num_least=50  `.
	+ The holes are stored in a public member variable `(paths)holes`.
+ `(void)NEPathPlanner::addholes()`: Add some new holes, i.e., the inner boundaries, onto the slice. The start point and the end point of every hole are connected by default. A slice is allowed to have no holes. The same as `(void)NEPathPlanner::set_contour()`, you can call `NEPathPlanner::tool_compensate()` to offset the added hole.
	+ `(const paths&)holes_new`: Add `path`s in `holes_new` onto the slice.
	+ `(bool)wash`, `(double)wash_dis`, `(int)num_least`: If `wash==true`, the contour would be resampled with a uniformly-distributed distance no more than `wash_dis`, and the number of waypoints are no less than `num_least`. By default, `wash=true, washdis=0.2, num_least=50  `.
	+ The holes are stored in a public member variable `(paths)holes`.
+ `(paths)NEPathPlanner::tool_compensate()`: Offset the contour and holes of the slice with a distance, i.e., tool compensating.
	+ `(const ContourParallelOptions&)opts`:
		+ The offsetting distance is `opts.delta`. If `opts.delta>0`, the contour will be offset outside and the holes will be offset inside. If `opts.delta<0`, the contour will be offset inside and the holes will be offset outside.
		+ If `opts.wash==true`, the contour would be resampled with a uniformly-distributed distance no more than `opts.wash_dis`, and the number of waypoints are no less than `opts.num_least`.
	+ The order of outputs is the offsetting results of `contour`, `holes[0]`, `holes[1]`, ..., `holes[holes.size()-1]`. Note that the offsetting results of each toolpath can be one, serval, or even zero toolpath.
	+  `(paths)NEPathPlanner::tool_compensate()` is achieved based on [AngusJohnson/Clipper2](https://github.com/AngusJohnson/Clipper2).
+ `(paths)NEPathPlanner::IQOP()`: Generate the **IQOP** toolpath of a slice. The optimization problem of IQOP is provided above. If you don't need IQOP and other optimization-based toolpaths, you can comment out `#define IncludeGurobi` in `NEPath-master/setup_NEPath.h` to avoid the dependence on [Gurobi](https://www.gurobi.com/).
  + `(const NonEquidistantOptions&)opts`: 
    + `opts.delta` is the maximum distance between toolpaths. `opts.alpha` the scale of the minimum distance. The distances between toolpaths at every point are between `opts.alpha*opts.delta` and `opts.delta`, i.e., $\forall i,\delta_i\in$ (`opts.alpha*opts.delta`, `opts.delta`).  `opts.dot_delta` is $\dot\delta_\mathrm{m}$, i.e., the upper bound of $\frac{\mathrm{d}\delta}{\mathrm{d}s}$. `opts.dot_delta` is $\ddot\delta_\mathrm{m}$, i.e., the upper bound of $\frac{\mathrm{d}^2\delta}{\mathrm{d}s^2}$.
    + `opts.optimize_Q` is true if $Q$ is in the objective function. `opts.optimize_S` is true if $S$ is in the objective function. `opts.optimize_L` is true if $L$ is in the objective function.  `opts.lambda_Q`,  `opts.lambda_S`, and `opts.lambda_L` are $\lambda_Q,\lambda_S,\lambda_L$, respectively.
    + `opts.epsilon` is the upper bound of error in $\left\|\cdot\right\|_\infty$. `opts.set_max` is the maximum iteration steps.
    + If `opts.wash==true`, the contour would be resampled with a uniformly-distributed distance no more than `opts.wash_dis`, and the number of waypoints are no less than `opts.num_least`.
    + If `opts.connect` is `none`, then toolpaths are not connected. If `opts.connect` is `cfs`/`dfs`, then toolpaths are connected into a continuous one based on CFS/DFS. 
+ `(paths)NEPathPlanner::Raster()`: Generate the **Raster** toolpath of a slice.
	+ `(const DirectParallelOptions&)opts`: `opts.delta` is the distance between toolpaths. `opts.angle` is the angle between Raster toolpaths and the $x$-axis. The unit of `opts.angle` is rad, and you can use `acos(-1.0)` to obtain a accurate $\pi=3.1415926\cdots$.
	+ Every Raster toolpath has two waypoints, i.e., the start point and the end point.
+ `(paths)NEPathPlanner::Zigzag()`: Generate the **Zigzag** toolpath of a slice.
	+ `(const DirectParallelOptions&)opts`: `opts.delta` is the distance between toolpaths. `opts.angle` is the angle between Zigzag toolpaths and the $x$-axis. The unit of `opts.angle` is rad, and you can use `acos(-1.0)` to obtain a accurate $\pi=3.1415926\cdots$.
	+ Every Zigzag toolpath has an even numbers of waypoints.
+ `(paths)NEPathPlanner::CP()`: Generate the **CP** toolpath of a slice.
	+ `(const ContourParallelOptions&)opts`: `opts.delta` is the distance between toolpaths. If `opts.wash==true`, the contour would be resampled with a uniformly-distributed distance no more than `opts.wash_dis`, and the number of waypoints are no less than `opts.num_least`.
	+ `(paths)NEPathPlanner::CP()` is achieved based on [AngusJohnson/Clipper2](https://github.com/AngusJohnson/Clipper2).
	+ If `opts.connect` is `none`, then toolpaths are not connected. If `opts.connect` is `cfs`/`dfs`, then toolpaths are connected into a continuous one based on CFS/DFS. 
+ Other toolpath generation algorithms and toolpath connection algorithm will be added into `NEPathPlanner` latter.

#### `NEPath-master/Curve.h`

`Curve.h` has some fundamental  methods on geometry.

+ **Underfill**. For a slice $D\subset\mathbb{R}^2$ and some toolpaths $\left\{l_i\right\}_{i=1}^N$, underfill is defined as $D\bigcap\left(\bigcup\limits_{i=1}^n B_{\frac{\delta}2}\left(l_i\right)\right)^C$, where $\delta>0$ is the line width.	
	
	+ `(static UnderFillSolution)Curve::UnderFill()`: API of calculate underfill. Return a `UnderFillSolution`.
	
		+ `(const path&)contour`: the contour of slice.
		+ `(const paths&)holes`: the holes of slice. If the slice has no hole, you can input `paths()` as an empty set of holes.
		+ `(const paths&)ps`: the toolpaths planned before.
		+ `(double)delta`: the line width $\delta$. Note that for every toolpath, only a width of $\frac\delta2$ on each side is determined as fill.
		+ `(double)reratio`: the resolution ratio. `xs` and `ys` are sampled with a distance of `reratio` between 2 points.
	
	+ `(struct)UnderFillSolution` is a struct to store information of underfill.
		
		+ `(double*)xs` and `(double*)ys` are discrete points on $x$-axis and $y$-axis.
		+ `(int)nx` and  `(int)ny` are the lengths of `xs` and `ys`.
		+ `(bool**)map_slice` stores information of slice $D$. `map_slice[i][j]==true` if and only if the point `(xs[i],ys[j])`$\in D$.
		+ `(bool**)map_delta` stores information of neighborhood of toolpaths $\bigcup\limits_{i=1}^n B_{\frac{\delta}2}\left(l_i\right)$. `map_delta[i][j]==true` if and only if the point `(xs[i],ys[j])`$\in\bigcup\limits_{i=1}^n B_{\frac{\delta}2}\left(l_i\right)$.
		+ `(double)underfillrate` is the underfill rate, i.e.,
<p align="center">
	<img src="https://github.com/user-attachments/assets/794ee62e-5d2c-4a19-9837-51e30517e343" alt="optimization_problem" height="80" />
</p>


+ **Sharp corner.** To avoid computational sensitivity, sharp corners are determined by [area invariant](https://doi.org/10.1016/j.cagd.2008.01.002) (Helmut Pottmann, et al. 2009).
+ `(static SharpTurnSolution)Curve::SharpTurn_Invariant()`: determine sharp corners on a toolpath:
	  + `(const path&)p`: the input toolpath.
	  + `(double)radius`: the radius of the rolling circle.
	  + `(double)threshold`: the threshold to determine a sharp corner.
	  + `(bool)close`: `close` is true if and only if the toolpath is closed.
	  + `(bool)washdis`: sharp corners would be determined with a uniformly-distributed distance no more than `washdis`.
	+ `(struct)SharpTurnSolution` is a struct to store information of sharp corners for a toolpath `p`.
		+ `(int)length`: length of the toolpath.


		+ `(double)radius`: the radius of the rolling circle.
		+ `(double)threshold`: the threshold to determine a sharp corner.
		+ `(double*)AreaPercent`: `AreaPercent[i]` is the percent of area on one side of the toolpath at `(p.x[i],p.y[i])`.
		+ `(bool*)SharpTurn`: `SharpTurn[i]==ture` if and only if `AreaPercent[i]>threshold`.
		+ `(bool)close`: `close` is true if and only if the toolpath is closed.

## Examples

### Toolpath Generation

#### IQOP (Isoperimetric-Quotient-Optimal Toolpath, Wang Y et al., 2023)

##### Based on Gurobi


```c++
#include <NEPath/NEPath.h>
#include <iostream>
using namespace std;
using namespace nepath; // the namspace of the NEPath package

int main() {
   	NEPathPlanner planner;

	// Obtain the contour of the outer boundary of slices
	path contour;
	contour.length = 1000; // the number of waypoints
	contour.x = new double[contour.length](); // x-coordinate of waypoints
	contour.y = new double[contour.length](); // y-coordinate of waypoints
	const double pi = acos(-1.0); // pi == 3.1415926...
	for (int i = 0; i < contour.length; ++i) {
		double theta = 2.0 * pi * i / contour.length;
		double r = 15.0 * (1.0 + 0.1 * cos(10.0 * theta));
		contour.x[i] = r * cos(theta);
		contour.y[i] = r * sin(theta);
	}

	// The out boundary should be offset with half of the line width to obtain the outmost toolpath
	NEPathPlanner planner_toolcompensate;
	planner_toolcompensate.set_contour(contour);
	ContourParallelOptions opts_toolcompensate;
	opts_toolcompensate.delta = -1.0 * 0.5; // half of the line width of toolpaths
	opts_toolcompensate.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts_toolcompensate.washdis = 0.2;
	paths path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate);

	planner.set_contour(path_outmost[0]);
	// or `planner.set_contour(contour.x, contour.y, contour.length)`

	// Set the toolpath parameters
	NonEquidistantOptions opts;
	opts.delta = 1.0; // the line width of toolpaths
	opts.alpha = 0.5; // the scale of minimum distance
	opts.dot_delta = 1.0; // the upper bound of \dot{delta_i}
	opts.ddot_delta = 0.1; // the upper bound of \ddot{delta_i}

	opts.optimize_Q = true; // the isoperimetric quotient is in the objective function
	opts.optimize_S = false; // the area is not in the objective function
	opts.optimize_L = false; // the length is not in the objective function
	opts.lambda_Q = 1.0; // the weighting coefficient of the isoperimetric quotient

	opts.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts.washdis = 0.2;
    opts.connector = ConnectAlgorithm::none; // don't connect in this example
    opts.optimizer = OptimizationAlgorithm::gurobi; // use Gurobi solver

	paths IQOP_paths = planner.IQOP(opts, true); // all IQOP paths
	cout << "There are " << IQOP_paths.size() << " continuous toolpaths in total." << endl;
	return 0;
}
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


```c++
#include <NEPath/NEPath.h>
#include <iostream>
using namespace std;
using namespace nepath; // the namspace of the NEPath package

int main() {
   	NEPathPlanner planner;

	// Obtain the contour of the outer boundary of slices
	path contour;
	contour.length = 1000; // the number of waypoints
	contour.x = new double[contour.length](); // x-coordinate of waypoints
	contour.y = new double[contour.length](); // y-coordinate of waypoints
	const double pi = acos(-1.0); // pi == 3.1415926...
	for (int i = 0; i < contour.length; ++i) {
		double theta = 2.0 * pi * i / contour.length;
		double r = 15.0 * (1.0 + 0.1 * cos(10.0 * theta));
		contour.x[i] = r * cos(theta);
		contour.y[i] = r * sin(theta);
	}

	// The out boundary should be offset with half of the line width to obtain the outmost toolpath
	NEPathPlanner planner_toolcompensate;
	planner_toolcompensate.set_contour(contour);
	ContourParallelOptions opts_toolcompensate;
	opts_toolcompensate.delta = -1.0 * 0.5; // half of the line width of toolpaths
	opts_toolcompensate.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts_toolcompensate.washdis = 0.2;
	paths path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate);

	planner.set_contour(path_outmost[0]);
	// or `planner.set_contour(contour.x, contour.y, contour.length)`

	// Set the toolpath parameters
	NonEquidistantOptions opts;
	opts.delta = 1.0; // the line width of toolpaths
	opts.alpha = 0.5; // the scale of minimum distance
	opts.dot_delta = 1.0; // the upper bound of \dot{delta_i}
	opts.ddot_delta = 0.1; // the upper bound of \ddot{delta_i}

	opts.optimize_Q = true; // the isoperimetric quotient is in the objective function
	opts.optimize_S = false; // the area is not in the objective function
	opts.optimize_L = false; // the length is not in the objective function
	opts.lambda_Q = 1.0; // the weighting coefficient of the isoperimetric quotient

	opts.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts.washdis = 0.2;
    opts.connector = ConnectAlgorithm::none; // don't connect in this example
    opts.optimizer = OptimizationAlgorithm::ipopt; // use Ipopt solver

	paths IQOP_paths = planner.IQOP(opts, true); // all IQOP paths
	cout << "There are " << IQOP_paths.size() << " continuous toolpaths in total." << endl;
	return 0;
}
```

<p align="center">
	<img src="https://github.com/user-attachments/assets/a7c03420-5131-446a-8f8b-6a6174281012" alt="IQOP-ipopt" height="300" />
</p>
<p align="center">
	<b>Figure.</b> IQOP toolpath minimizing L based on ipopt.
</p>

#### CP (Contour-Parallel)

```c++
#include <NEPath/NEPath.h>
#include <iostream>
using namespace std;
using namespace nepath; // the namspace of the NEPath package

int main() {
	NEPathPlanner planner;

	// Obtain the contour of the outer boundary of slices
	path contour;
	contour.length = 1000; // the number of waypoints
	contour.x = new double[contour.length](); // x-coordinate of waypoints
	contour.y = new double[contour.length](); // y-coordinate of waypoints
	const double pi = acos(-1.0); // pi == 3.1415926...
	for (int i = 0; i < contour.length; ++i) {
		double theta = 2.0 * pi * i / contour.length;
		double r = 15.0 * (1.0 + 0.15 * cos(10.0 * theta));
		contour.x[i] = r * cos(theta);
		contour.y[i] = r * sin(theta);
	}

	// The out boundary should be offset with half of the line width to obtain the outmost toolpath
	NEPathPlanner planner_toolcompensate;
	planner_toolcompensate.set_contour(contour);
	ContourParallelOptions opts_toolcompensate;
	opts_toolcompensate.delta = -1.0 * 0.5; // half of the line width of toolpaths
	opts_toolcompensate.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts_toolcompensate.washdis = 0.2;
	paths path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate);

	planner.set_contour(path_outmost[0]);
	// or `planner.set_contour(contour.x, contour.y, contour.length)`

	// Set the toolpath parameters
	ContourParallelOptions opts;
	opts.delta = 1.0; // the line width of toolpaths
	opts.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts.washdis = 0.2;

	paths CP_paths = planner.CP(opts); // all CP paths
	cout << "There are " << CP_paths.size() << " continuous toolpaths in total." << endl;
	for (int i = 0; i < CP_paths.size(); ++i) {
		// CP_paths[i] is the i-th continuous toolpath
		cout << "Toopath " << i << " has " << CP_paths[i].length << " waypoints." << endl;
	}
	
	return 0;
}
```

<p align="center">
	<img src="https://user-images.githubusercontent.com/75420225/229331109-58155b93-d897-4553-b923-28be4eecfee1.png" alt="CP" height="300" />
</p>
<p align="center">
	<b>Figure.</b> CP toolpath.
</p>


#### Zigzag

```c++
#include <NEPath/NEPath.h>
#include <iostream>
using namespace std;
using namespace nepath; // the namspace of the NEPath package

int main() {
	NEPathPlanner planner;

	// Set the contour
	path contour;
	contour.length = 1000; // the number of waypoints
	contour.x = new double[contour.length](); // x-coordinate of waypoints
	contour.y = new double[contour.length](); // y-coordinate of waypoints
	const double pi = acos(-1.0); // pi == 3.1415926...
	for (int i = 0; i < contour.length; ++i) {
		double theta = 2.0 * pi * i / contour.length;
		double r = 15.0 * (1.0 + 0.15 * cos(10.0 * theta));
		contour.x[i] = r * cos(theta);
		contour.y[i] = r * sin(theta);
	}
	planner.set_contour(contour);
	// or `planner.set_contour(contour.x, contour.y, contour.length)`

	// Set the toolpath parameters
	DirectParallelOptions opts;
	opts.delta = 1.0; // the line width of toolpaths
	opts.angle = pi / 3.0; // the angle of Zigzag toolpaths, unit: rad

	paths zigzag_paths = planner.Zigzag(opts); // all zigzag paths
	cout << "There are " << zigzag_paths.size() << " continuous toolpaths in total." << endl;
	for (int i = 0; i < zigzag_paths.size(); ++i) {
		// zigzag_paths[i] is the i-th continuous toolpath
		cout << "Toopath " << i << " has " << zigzag_paths[i].length << " waypoints." << endl;
	}
	
	return 0;
}
```

<p align="center">
	<img src="https://user-images.githubusercontent.com/75420225/229331070-887c0172-7ad4-42dd-85a1-6c35f09a6cc4.png" alt="zigzag" height="300" />
</p>
<p align="center">
	<b>Figure.</b> Zigzag toolpath.
</p>


#### Raster

```c++
#include <NEPath/NEPath.h>
#include <iostream>
using namespace std;
using namespace nepath; // the namspace of the NEPath package

int main() {
	NEPathPlanner planner;

	// Set the contour
	path contour;
	contour.length = 1000; // the number of waypoints
	contour.x = new double[contour.length](); // x-coordinate of waypoints
	contour.y = new double[contour.length](); // y-coordinate of waypoints
	const double pi = acos(-1.0); // pi == 3.1415926...
	for (int i = 0; i < contour.length; ++i) {
		double theta = 2.0 * pi * i / contour.length;
		double r = 15.0 * (1.0 + 0.15 * cos(10.0 * theta));
		contour.x[i] = r * cos(theta);
		contour.y[i] = r * sin(theta);
	}
	planner.set_contour(contour);
	// or `planner.set_contour(contour.x, contour.y, contour.length)`

	// Set the toolpath parameters
	DirectParallelOptions opts;
	opts.delta = 1.0; // the line width of toolpaths
	opts.angle = - pi / 3.0; // the angle of raster toolpaths, unit: rad

	paths raster_paths = planner.Raster(opts); // all raster paths
	cout << "There are " << raster_paths.size() << " continuous toolpaths in total." << endl;
	
	return 0;
}
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

```c++
#include <NEPath/NEPath.h>
#include <iostream>
using namespace std;
using namespace nepath; // the namspace of the NEPath package

int main() {
    NEPathPlanner planner;

    // Obtain the contour of the outer boundary of slices
    path contour;
    contour.length = 1000; // the number of waypoints
    contour.x = new double[contour.length](); // x-coordinate of waypoints
    contour.y = new double[contour.length](); // y-coordinate of waypoints
    const double pi = acos(-1.0); // pi == 3.1415926...
    for (int i = 0; i < contour.length; ++i) {
        double theta = 2.0 * pi * i / contour.length;
        double r = 15.0 * (1.0 + 0.1 * cos(10.0 * theta));
        contour.x[i] = r * cos(theta);
        contour.y[i] = r * sin(theta);
    }

    // The out boundary should be offset with half of the line width to obtain the outmost toolpath
    NEPathPlanner planner_toolcompensate;
    planner_toolcompensate.set_contour(contour);
    ContourParallelOptions opts_toolcompensate;
    opts_toolcompensate.delta = -1.0 * 0.5; // half of the line width of toolpaths
    opts_toolcompensate.wash = true; // it is recommended to set opt.wash=true
    // if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
    opts_toolcompensate.washdis = 0.2;
    paths path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate);

    planner.set_contour(path_outmost[0]);
    // or `planner.set_contour(contour.x, contour.y, contour.length)`

    // Set the toolpath parameters
    NonEquidistantOptions opts;
    opts.delta = 1.0; // the line width of toolpaths
    opts.alpha = 0.5; // the scale of minimum distance
    opts.dot_delta = 1.0; // the upper bound of \dot{delta_i}
    opts.ddot_delta = 0.1; // the upper bound of \ddot{delta_i}

    opts.optimize_Q = true; // the isoperimetric quotient is in the objective function
    opts.optimize_S = false; // the area is not in the objective function
    opts.optimize_L = false; // the length is not in the objective function
    opts.lambda_Q = 1.0; // the weighting coefficient of the isoperimetric quotient

    opts.wash = true; // it is recommended to set opt.wash=true
    // if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
    opts.washdis = 0.2;
    opts.connect = ConnectAlgorithm::cfs; // select cfs as the connecting method
    opts.optimizer = OptimizationAlgorithm::gurobi; // use Gurobi solver

    paths IQOP_paths = planner.IQOP(opts, true); // IQOP with CFS
	return 0;
}
```

<p align="center">
	<img src="https://github.com/user-attachments/assets/38b01606-ff3e-4149-89c8-39335ed70ac3" alt="iqop_cfs" height="300" />
</p>
<p align="center">
	<b>Figure.</b> IQOP connected by CFS.
</p>

#### IQOP connected by DFS

CP can be connected by DFS in the same way.

```c++
#include <NEPath/NEPath.h>
#include <iostream>
using namespace std;
using namespace nepath; // the namspace of the NEPath package

int main() {
    NEPathPlanner planner;

    // Obtain the contour of the outer boundary of slices
    path contour;
    contour.length = 1000; // the number of waypoints
    contour.x = new double[contour.length](); // x-coordinate of waypoints
    contour.y = new double[contour.length](); // y-coordinate of waypoints
    const double pi = acos(-1.0); // pi == 3.1415926...
    for (int i = 0; i < contour.length; ++i) {
        double theta = 2.0 * pi * i / contour.length;
        double r = 15.0 * (1.0 + 0.1 * cos(10.0 * theta));
        contour.x[i] = r * cos(theta);
        contour.y[i] = r * sin(theta);
    }

    // The out boundary should be offset with half of the line width to obtain the outmost toolpath
    NEPathPlanner planner_toolcompensate;
    planner_toolcompensate.set_contour(contour);
    ContourParallelOptions opts_toolcompensate;
    opts_toolcompensate.delta = -1.0 * 0.5; // half of the line width of toolpaths
    opts_toolcompensate.wash = true; // it is recommended to set opt.wash=true
    // if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
    opts_toolcompensate.washdis = 0.2;
    paths path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate);

    planner.set_contour(path_outmost[0]);
    // or `planner.set_contour(contour.x, contour.y, contour.length)`

    // Set the toolpath parameters
    NonEquidistantOptions opts;
    opts.delta = 1.0; // the line width of toolpaths
    opts.alpha = 0.5; // the scale of minimum distance
    opts.dot_delta = 1.0; // the upper bound of \dot{delta_i}
    opts.ddot_delta = 0.1; // the upper bound of \ddot{delta_i}

    opts.optimize_Q = true; // the isoperimetric quotient is in the objective function
    opts.optimize_S = false; // the area is not in the objective function
    opts.optimize_L = false; // the length is not in the objective function
    opts.lambda_Q = 1.0; // the weighting coefficient of the isoperimetric quotient

    opts.wash = true; // it is recommended to set opt.wash=true
    // if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
    opts.washdis = 0.2;
    opts.connect = ConnectAlgorithm::dfs; // select dfs as the connecting method
    opts.optimizer = OptimizationAlgorithm::gurobi; // use Gurobi solver

    paths IQOP_paths = planner.IQOP(opts, true); // IQOP with DFS
	return 0;
}
```

<p align="center">
	<img src="https://github.com/user-attachments/assets/c03acb4f-c14f-40b9-81f5-ad49f758611d" alt="iqop_dfs" height="300" />
</p>
<p align="center">
	<b>Figure.</b> IQOP connected by DFS.
</p>
### Others

#### Tool  compensate

```c++
#include <NEPath/NEPath.h>
#include <iostream>
using namespace std;
using namespace nepath; // the namspace of the NEPath package

int main() {
	NEPathPlanner planner;

	// Obtain the contour of the outer boundary of slices
	path contour;
	contour.length = 1000; // the number of waypoints
	contour.x = new double[contour.length](); // x-coordinate of waypoints
	contour.y = new double[contour.length](); // y-coordinate of waypoints
	const double pi = acos(-1.0); // pi == 3.1415926...
	for (int i = 0; i < contour.length; ++i) {
		double theta = 2.0 * pi * i / contour.length;
		double r = 15.0 * (1.0 + 0.15 * cos(10.0 * theta));
		contour.x[i] = r * cos(theta);
		contour.y[i] = r * sin(theta);
	}
	planner.set_contour(contour);

	// Obtain the hole
	double x_hole[] = { -5,5,5,0,-5 };
	double y_hole[] = { -5,-5,5,0,5 };
	planner.addhole(x_hole, y_hole, 5);

	// Tool compensate
	ContourParallelOptions opts;
	opts.delta = -1.5; // the offset distance
	opts.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts.washdis = 0.2;
	paths ps_toolcompensate = planner.tool_compensate(opts); // Tool compensate

	cout << "There are " << ps_toolcompensate.size() << " continuous toolpaths in total." << endl;
	for (int i = 0; i < ps_toolcompensate.size(); ++i) {
		// ps_toolcompensate[i] is the i-th continuous toolpath
		cout << "Toopath " << i << " has " << ps_toolcompensate[i].length << " waypoints." << endl;
	}
	
	return 0;
}
```

<p align="center">
	<img src="https://user-images.githubusercontent.com/75420225/229334415-da87d12b-8c18-4a2b-b646-a8fe7558b06e.png" alt="Tool compensate" height="300" />
</p>
<p align="center">
	<b>Figure.</b> Tool compensate.
</p>


#### Underfill

```c++
#include <NEPath/NEPath.h>
#include <iostream>
using namespace std;
using namespace nepath; // the namspace of the NEPath package

int main() {
	NEPathPlanner planner;

	// Obtain the contour of the outer boundary of slices
	path contour;
	contour.length = 1000; // the number of waypoints
	contour.x = new double[contour.length](); // x-coordinate of waypoints
	contour.y = new double[contour.length](); // y-coordinate of waypoints
	const double pi = acos(-1.0); // pi == 3.1415926...
	for (int i = 0; i < contour.length; ++i) {
		double theta = 2.0 * pi * i / contour.length;
		double r = 15.0 * (1.0 + 0.15 * cos(10.0 * theta));
		contour.x[i] = r * cos(theta);
		contour.y[i] = r * sin(theta);
	}

	// The out boundary should be offset with half of the line width to obtain the outmost toolpath
	NEPathPlanner planner_toolcompensate;
	planner_toolcompensate.set_contour(contour);
	ContourParallelOptions opts_toolcompensate;
	opts_toolcompensate.delta = -1.0 * 0.5; // half of the line width of toolpaths
	opts_toolcompensate.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts_toolcompensate.washdis = 0.2;
	paths path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate);

	planner.set_contour(path_outmost[0]);
	// or `planner.set_contour(contour.x, contour.y, contour.length)`

	// Set the toolpath parameters
	ContourParallelOptions opts;
	opts.delta = 1.0; // the line width of toolpaths
	opts.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts.washdis = 0.2;

	paths CP_paths = planner.CP(opts); // all CP paths

	double delta_underfill = opts.delta; // the line width for underfill computation
	double reratio = 0.03; // resolution ratio for underfill computation

	UnderFillSolution ufs = Curve::UnderFill(contour, paths(), CP_paths, delta_underfill, reratio); // Obtain the results of underfill

	cout << "The underfill rate is " << ufs.underfillrate * 100 << "%." << endl;
	
	return 0;
}
```

<p align="center">
	<img src="https://github.com/WangY18/NEPath/assets/75420225/e050a611-8108-4f4d-b293-b801443de746" alt="Underfill" height="300" />
</p>
<p align="center">
    <b>Figure.</b> Underfill. The underfill rate is 1.2401% in this example.
</p>


#### Sharp corner

```c++
#include <NEPath/NEPath.h>
#include <iostream>
using namespace std;
using namespace nepath; // the namspace of the NEPath package

int main() {
	NEPathPlanner planner;

	// Obtain the contour of the outer boundary of slices
	path contour;
	contour.length = 1000; // the number of waypoints
	contour.x = new double[contour.length](); // x-coordinate of waypoints
	contour.y = new double[contour.length](); // y-coordinate of waypoints
	const double pi = acos(-1.0); // pi == 3.1415926...
	for (int i = 0; i < contour.length; ++i) {
		double theta = 2.0 * pi * i / contour.length;
		double r = 15.0 * (1.0 + 0.15 * cos(10.0 * theta));
		contour.x[i] = r * cos(theta);
		contour.y[i] = r * sin(theta);
	}

	// The out boundary should be offset with half of the line width to obtain the outmost toolpath
	NEPathPlanner planner_toolcompensate;
	planner_toolcompensate.set_contour(contour);
	ContourParallelOptions opts_toolcompensate;
	opts_toolcompensate.delta = -1.0 * 0.5; // half of the line width of toolpaths
	opts_toolcompensate.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts_toolcompensate.washdis = 0.2;
	paths path_outmost = planner_toolcompensate.tool_compensate(opts_toolcompensate);

	planner.set_contour(path_outmost[0]);
	// or `planner.set_contour(contour.x, contour.y, contour.length)`

	// Set the toolpath parameters
	ContourParallelOptions opts;
	opts.delta = 1.0; // the line width of toolpaths
	opts.wash = true; // it is recommended to set opt.wash=true
	// if wash==true, then all toolpaths would have yniformly distributed waypoints, with a distance near opts.washdis
	opts.washdis = 0.2;

	paths CP_paths = planner.CP(opts); // all CP paths

	double radius = 1.0; // radius of the rolling circle
	double threshold = 0.3; // threshold of area on one side to determine a sharp corner

	// Obtain the results of underfill
	int num = 0;
	for (int i = 0; i < CP_paths.size(); ++i) {
		SharpTurnSolution sol = Curve::SharpTurn_Invariant(CP_paths[i], radius, threshold, true, 0.5);
		for (int j = 0; j < sol.length; ++j) {
			num += sol.SharpTurn[j];
		}
	}

	cout << "There exist " << num << " sharp corners." << endl;
	
	return 0;
}
```

<p align="center">
	<img src="https://github.com/WangY18/NEPath/assets/75420225/4586213c-d5e2-415b-b81a-d09ed680ffe9" alt="Underfill" height="300" />
</p>
<p align="center">
    <b>Figure.</b> Sharp corners. There exist 44 sharp corners in this example.
</p>
