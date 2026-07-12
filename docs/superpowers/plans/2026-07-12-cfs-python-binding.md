# CFS Python Binding Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose the existing connected Fermat spiral algorithm for validated caller-supplied contour trees through Python.

**Architecture:** A binding-local C++ component validates geometry and topology, builds the legacy `pathnode` tree, invokes `Connector::ConnectedFermatSpiral_MultMinimum`, and owns the surviving root. `nepath_py.cpp` only registers that component. Python tests exercise the compiled extension and the tutorial demonstrates arbitrary input ordering.

**Tech Stack:** C++17, nanobind, Python 3.12, NumPy, pytest, CMake, pixi.

## Global Constraints

- Work on `cfs-python-binding`, based on `main`; do not modify IQOP PR #8.
- Support one general containment tree with exactly one outer root.
- Validate all inputs before allocating `pathnode` objects.
- Use named exceptions for every public failure mode.
- Use pixi exclusively for configuration, builds, formatting, and tests.
- Run pytest with `-n auto --testmon`.
- Keep `nepath_py.cpp` registration-only for this feature.

---

### Task 1: Reproducible failing Python contract

**Files:**
- Modify: `bindings/python/pyproject.toml`
- Create: `bindings/python/tests/test_connect_fermat_spiral.py`
- Modify: `bindings/python/NEPath/nepath_py.cpp`

**Interfaces:**
- Consumes: installed `NEPath.Path.from_arrays` and `Path.get_arrays`.
- Produces: a failing contract for `NEPath.connect_fermat_spiral`.

- [ ] Remove the untested inline CFS draft from `nepath_py.cpp`, restoring registration to its `main` state.
- [ ] Add `[tool.pixi.workspace]`, dependencies for CMake/Ninja/compiler/nanobind/NumPy/pytest/pytest-xdist/pytest-testmon/ruff, and tasks that configure and build an optimizer-disabled extension.
- [ ] Add a test helper:

```python
def square(radius: float, center_x: float = 0.0) -> nepath.Path:
    x = np.array([center_x - radius, center_x + radius, center_x + radius, center_x - radius])
    y = np.array([-radius, -radius, radius, radius])
    return nepath.Path.from_arrays(x, y)
```

- [ ] Add `test_connect_fermat_spiral_is_exposed` asserting `hasattr(nepath, "connect_fermat_spiral")`.
- [ ] Run `pixi run test-cfs`; expect one assertion failure because the API is absent.

### Task 2: Validated CFS binding component

**Files:**
- Create: `bindings/python/NEPath/cfs_binding.h`
- Create: `bindings/python/NEPath/cfs_binding.cpp`
- Modify: `bindings/python/NEPath/nepath_py.cpp`
- Modify: `bindings/python/CMakeLists.txt`
- Modify: `bindings/python/tests/test_connect_fermat_spiral.py`

**Interfaces:**
- Produces: `void register_cfs_binding(nanobind::module_&)`.
- Exposes: `connect_fermat_spiral(contours: list[Path], spacing: float) -> Path`.
- Exposes: `CfsBindingError`, `EmptyCfsContoursError`, `InvalidCfsSpacingError`, `InvalidCfsContourError`, and `InvalidCfsTopologyError`.

- [ ] Add failing tests for an arbitrary-order nested chain and a branching tree; require a non-empty result containing only finite coordinates.
- [ ] Add failing tests for empty input, zero/NaN spacing, a two-point contour, a contour with NaN, a zero-area contour, and two disjoint roots; assert the specific named exception.
- [ ] Run `pixi run test-cfs`; confirm failures are missing API/exception symbols.
- [ ] Implement exception types and `register_cfs_binding` declarations in `cfs_binding.h`.
- [ ] Implement pre-allocation validation in `cfs_binding.cpp`: finite positive spacing, at least three finite vertices, nonzero extent/edges/area using a named machine-epsilon-derived geometric floor, parent-index derivation by smallest containing larger contour, and exactly one root.
- [ ] Allocate nodes only after validation, release connector-consumed child ownership, keep the root in `std::unique_ptr`, copy the returned path, and delete the root on scope exit.
- [ ] Register named exceptions and `connect_fermat_spiral` in `register_cfs_binding`.
- [ ] Add `NEPath/cfs_binding.cpp` to `nanobind_add_module` and call `register_cfs_binding(m)` from `nepath_py.cpp`.
- [ ] Run `pixi run test-cfs`; expect all CFS tests to pass.

### Task 3: Stub and tutorial consumer contracts

**Files:**
- Modify: `bindings/python/NEPath/_nepath.pyi`
- Modify: `tutorial/python.md`
- Modify: `bindings/python/tests/test_connect_fermat_spiral.py`

**Interfaces:**
- Documents and types the exact runtime API from Task 2.

- [ ] Add the five exception classes and typed function declaration to `_nepath.pyi`:

```python
class CfsBindingError(RuntimeError): ...
class EmptyCfsContoursError(CfsBindingError): ...
class InvalidCfsSpacingError(CfsBindingError): ...
class InvalidCfsContourError(CfsBindingError): ...
class InvalidCfsTopologyError(CfsBindingError): ...

def connect_fermat_spiral(contours: list[Path], spacing: float) -> Path: ...
```

- [ ] Add a tutorial section that creates three square `Path` objects, deliberately scrambles their order, calls `nepath.connect_fermat_spiral(contours, spacing=1.0)`, and plots the returned path.
- [ ] Add a test that reads the stub and tutorial, asserting the function declaration and executable example call remain present.
- [ ] Run `pixi run test-cfs`; expect all runtime and documentation contracts to pass.

### Task 4: Verification and delivery

**Files:**
- Remove before delivery: `docs/superpowers/specs/2026-07-12-cfs-python-binding-design.md`
- Remove before delivery: `docs/superpowers/plans/2026-07-12-cfs-python-binding.md`

**Interfaces:**
- Produces: a focused CFS binding branch with no internal planning artefacts.

- [ ] Run C++ formatting on changed C++ files and Ruff on changed Python files.
- [ ] Run `pixi run test-cfs` and the existing Python example tests affected by the binding.
- [ ] Run `git diff --check` and inspect the branch diff against `main`.
- [ ] Remove the internal spec and plan files, commit their deletion, and verify they do not appear in the final PR file list.
- [ ] Commit only the binding, build contract, tests, stub, and tutorial using Jelle Feringa as author and committer.
- [ ] Push `cfs-python-binding` and open a separate PR against `WangY18/NEPath:main`.
