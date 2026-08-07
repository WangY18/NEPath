from __future__ import annotations

from pathlib import Path

import NEPath as nepath
import numpy as np
import pytest

EXCESSIVE_INPUT_VERTEX_COUNT = 5_001
MAXIMUM_ACCEPTED_INPUT_VERTEX_COUNT = 5_000


def square(radius: float, center_x: float = 0.0, center_y: float = 0.0) -> nepath.Path:
    x = np.array(
        [
            center_x - radius,
            center_x + radius,
            center_x + radius,
            center_x - radius,
        ],
        dtype=np.float64,
    )
    y = np.array(
        [
            center_y - radius,
            center_y - radius,
            center_y + radius,
            center_y + radius,
        ],
        dtype=np.float64,
    )
    return nepath.Path.from_arrays(x, y)


def polygon(vertices: list[tuple[float, float]]) -> nepath.Path:
    coordinates = np.asarray(vertices, dtype=np.float64)
    return nepath.Path.from_arrays(coordinates[:, 0], coordinates[:, 1])


def wavy_contour(radius: float) -> nepath.Path:
    theta = np.linspace(0.0, 2.0 * np.pi, 1000, endpoint=False)
    modulated_radius = radius * (1.0 + 0.1 * np.cos(10.0 * theta))
    return nepath.Path.from_arrays(modulated_radius * np.cos(theta), modulated_radius * np.sin(theta))


def test_connect_fermat_spiral_is_exposed() -> None:
    assert hasattr(nepath, "connect_fermat_spiral")


def test_path_length_is_read_only() -> None:
    contour = square(3.0)

    with pytest.raises(AttributeError):
        contour.length = 100


def assert_finite_nonempty(path: nepath.Path) -> None:
    x, y = path.get_arrays()
    assert len(x) == len(y)
    assert len(x) > 0
    assert np.all(np.isfinite(x))
    assert np.all(np.isfinite(y))


def test_connects_arbitrary_order_nested_chain() -> None:
    contours = [square(3.0), square(9.0), square(6.0)]

    result = nepath.connect_fermat_spiral(contours, spacing=1.0)

    assert_finite_nonempty(result)


def test_tutorial_wavy_contours_connect_without_optimizer() -> None:
    contours = [wavy_contour(10.0), wavy_contour(15.0), wavy_contour(5.0)]

    result = nepath.connect_fermat_spiral(contours, spacing=1.0)

    assert_finite_nonempty(result)


def test_connects_branching_containment_tree() -> None:
    contours = [square(2.0, 3.0), square(10.0), square(2.0, -3.0)]

    result = nepath.connect_fermat_spiral(contours, spacing=1.0)

    assert_finite_nonempty(result)


def test_connects_small_parent_without_revisiting_contour() -> None:
    contours = [square(0.05), square(0.1)]

    result = nepath.connect_fermat_spiral(contours, spacing=0.5)

    assert_finite_nonempty(result)


def test_connects_with_spacing_larger_than_contour_perimeters() -> None:
    contours = [square(3.0), square(9.0)]

    result = nepath.connect_fermat_spiral(contours, spacing=np.sqrt(np.finfo(np.float64).max))

    assert_finite_nonempty(result)


def test_rejects_empty_contour_collection() -> None:
    with pytest.raises(nepath.EmptyCfsContoursError):
        nepath.connect_fermat_spiral([], spacing=1.0)


@pytest.mark.parametrize("spacing", [0.0, -1.0, np.nan, np.inf])
def test_rejects_invalid_spacing(spacing: float) -> None:
    with pytest.raises(nepath.InvalidCfsSpacingError):
        nepath.connect_fermat_spiral([square(3.0)], spacing=spacing)


def test_rejects_spacing_that_exceeds_sampling_budget() -> None:
    with pytest.raises(nepath.InvalidCfsSpacingError, match="waypoint safety limit"):
        nepath.connect_fermat_spiral([square(3.0)], spacing=np.finfo(np.float64).tiny)


def test_rejects_contour_with_too_few_vertices() -> None:
    invalid = nepath.Path.from_arrays(
        np.array([0.0, 1.0], dtype=np.float64),
        np.array([0.0, 1.0], dtype=np.float64),
    )

    with pytest.raises(nepath.InvalidCfsContourError):
        nepath.connect_fermat_spiral([invalid], spacing=1.0)


def test_rejects_nonfinite_contour() -> None:
    invalid = square(3.0)
    invalid.set_arrays(
        np.array([np.nan, 3.0, 3.0, -3.0], dtype=np.float64),
        np.array([-3.0, -3.0, 3.0, 3.0], dtype=np.float64),
    )

    with pytest.raises(nepath.InvalidCfsContourError):
        nepath.connect_fermat_spiral([invalid], spacing=1.0)


def test_rejects_zero_area_contour() -> None:
    invalid = nepath.Path.from_arrays(
        np.array([0.0, 1.0, 2.0], dtype=np.float64),
        np.array([0.0, 0.0, 0.0], dtype=np.float64),
    )

    with pytest.raises(nepath.InvalidCfsContourError):
        nepath.connect_fermat_spiral([invalid], spacing=1.0)


def test_rejects_multiple_outer_roots() -> None:
    contours = [square(1.0, -5.0), square(1.0, 5.0)]

    with pytest.raises(nepath.InvalidCfsTopologyError):
        nepath.connect_fermat_spiral(contours, spacing=1.0)


def test_rejects_touching_sibling_contours() -> None:
    contours = [square(2.0, -2.0), square(10.0), square(2.0, 2.0)]

    with pytest.raises(nepath.InvalidCfsTopologyError, match="intersect or touch"):
        nepath.connect_fermat_spiral(contours, spacing=1.0)


def test_rejects_child_edges_crossing_concave_parent_boundary() -> None:
    concave_parent = polygon([(-5.0, -5.0), (5.0, -5.0), (5.0, 5.0), (2.0, 5.0), (2.0, -2.0), (-2.0, -2.0), (-2.0, 5.0), (-5.0, 5.0)])
    crossing_child = polygon([(-4.0, 4.0), (4.0, 4.0), (0.0, -4.0)])

    with pytest.raises(nepath.InvalidCfsTopologyError):
        nepath.connect_fermat_spiral([crossing_child, concave_parent], spacing=1.0)


def test_rejects_self_intersecting_contour() -> None:
    self_intersecting = polygon([(0.0, 0.0), (3.0, 3.0), (0.0, 3.0), (3.0, 0.0), (1.5, -1.0)])

    with pytest.raises(nepath.InvalidCfsContourError, match="self-intersection"):
        nepath.connect_fermat_spiral([self_intersecting], spacing=1.0)


def test_rejects_input_before_quadratic_validation_becomes_unbounded() -> None:
    theta = np.linspace(0.0, 2.0 * np.pi, EXCESSIVE_INPUT_VERTEX_COUNT, endpoint=False)
    excessive = nepath.Path.from_arrays(np.cos(theta), np.sin(theta))

    with pytest.raises(nepath.InvalidCfsContourError, match="simplify"):
        nepath.connect_fermat_spiral([excessive], spacing=1.0)


def test_rejects_contour_family_exceeding_geometry_comparison_budget() -> None:
    theta = np.linspace(0.0, 2.0 * np.pi, MAXIMUM_ACCEPTED_INPUT_VERTEX_COUNT, endpoint=False)
    contours = [nepath.Path.from_arrays(radius * np.cos(theta), radius * np.sin(theta)) for radius in (1.0, 2.0, 3.0)]

    with pytest.raises(nepath.InvalidCfsTopologyError, match="boundary comparisons"):
        nepath.connect_fermat_spiral(contours, spacing=1.0)


def test_stub_and_tutorial_document_direct_cfs_contract() -> None:
    repository_root = Path(__file__).resolve().parents[3]
    stub = (repository_root / "bindings/python/NEPath/_nepath.pyi").read_text(encoding="utf-8")
    tutorial = (repository_root / "tutorial/python.md").read_text(encoding="utf-8")

    assert "def connect_fermat_spiral(contours: list[Path], spacing: float) -> Path:" in stub
    assert "connected_path = nepath.connect_fermat_spiral(" in tutorial
    assert "contours_arbitrary_order" in tutorial
