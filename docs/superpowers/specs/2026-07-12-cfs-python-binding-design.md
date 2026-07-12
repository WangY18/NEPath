# CFS Python binding design

## Goal

Expose NEPath's existing connected Fermat spiral implementation for caller-supplied nested `Path` contours without coupling the change to the IQOP parity PR.

## Public API

```python
connect_fermat_spiral(contours: list[Path], spacing: float) -> Path
```

`contours` may be in arbitrary order. They must form one containment tree with exactly one outer root; sibling and nested child contours are supported. `spacing` is the positive finite toolpath spacing passed to `Connector::ConnectedFermatSpiral_MultMinimum`.

## Components

- A binding-local C++ component owns validation, containment-tree construction, and connector invocation.
- `nepath_py.cpp` only registers the function and named exceptions.
- The Python stub exposes the typed function and exception hierarchy.
- `tutorial/python.md` contains a runnable arbitrary-order contour example.

## Validation and errors

Validation occurs before allocating `pathnode` objects:

- `EmptyCfsContoursError`: no contours;
- `InvalidCfsSpacingError`: non-finite or non-positive spacing;
- `InvalidCfsContourError`: fewer than three vertices, null storage, non-finite coordinates, zero-length edges, or zero signed area;
- `InvalidCfsTopologyError`: the contours do not form exactly one containment tree.

The binding registers these named C++ exceptions for Python consumers.

## Ownership

Parent indices are derived before node allocation. The legacy connector consumes non-root nodes while combining the tree. The wrapper retains ownership of the surviving root, copies the returned `Path`, then deletes the root. Validation failures allocate no nodes.

## Tests

Python contract tests exercise the compiled extension with:

- a valid arbitrary-order nested chain;
- a valid branching containment tree;
- finite, non-empty output;
- empty input;
- invalid spacing;
- invalid contour geometry;
- multiple outer roots.

The implementation follows red-green-refactor: tests must fail because the API is absent before production code is added.

## Delivery

The change ships from `cfs-python-binding`, based on `main`, as a separate PR. It does not modify IQOP PR #8.
