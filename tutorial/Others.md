# NEPath: Other Notes

## Install Ipopt in Conda

```shell
conda install -c conda-forge ipopt
```

## Set Environment Variables in Terminal

### Compile

```shell
# Example for windows
$env:IPOPT_ROOT="C:\ProgramData\anaconda3\envs\NEPath\Library"
$env:GUROBI_VERSION="120"
# Example for linux / macOS
export IPOPT_ROOT=/home/robot/miniconda3/envs/NEPath
export GUROBI_VERSION=130
```

## Install Nanobind

```shell
git clone --recursive https://github.com/wjakob/nanobind.git
cd nanobind
mkdir build
cd build
```

### For Linux / macOS

```shell
cmake .. -DNANOBIND_INSTALL=ON -DBUILD_SHARED_LIBS=ON
# Or in conda: cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DNANOBIND_INSTALL=ON -DBUILD_SHARED_LIBS=ON
cmake --build . -j$(nproc)
cmake --install .
```

### For Windows

```shell
cmake .. -G "Visual Studio 17 2022" -A x64 -DNANOBIND_INSTALL=ON -DBUILD_SHARED_LIBS=ON 
# For conda: -DCMAKE_INSTALL_PREFIX="$env:CONDA_PREFIX"
cmake --build . --config Release
cmake --install . --config Release
```





```shell
cmake --build . --target install --config Release
```

