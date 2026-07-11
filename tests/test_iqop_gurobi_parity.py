from __future__ import annotations

import csv
import math
import subprocess
from dataclasses import dataclass
from pathlib import Path

import gurobipy as gp
import pytest
from gurobipy import GRB


SOLVER_TOLERANCE = 1.0e-8
FEASIBILITY_GATE = 10.0 * SOLVER_TOLERANCE
OBJECTIVE_RELATIVE_GATE = 2.0e-5  # L-BFGS Ipopt versus conic barrier optimum.
GEOMETRY_RELATIVE_GATE = 2.0e-5
OFFSET_INFINITY_GATE = 2.0e-4

GEOMETRY_FIXTURES: dict[str, tuple[tuple[float, ...], tuple[float, ...]]] = {
    "irregular": (
        (0.0, 3.0, 3.5, 1.8, 1.0, -0.4),
        (0.0, 0.2, 2.4, 1.4, 3.1, 1.8),
    ),
    "convex": (
        (0.0, 4.0, 4.5, 4.0, 2.5, 0.5, -0.5, 0.0),
        (0.0, 0.0, 1.0, 3.0, 4.0, 3.5, 2.0, 0.7),
    ),
    "slender": (
        (0.0, 5.0, 6.0, 5.2, 3.0, 0.5, -0.5),
        (0.0, 0.2, 1.0, 1.8, 2.2, 1.8, 1.0),
    ),
}


@dataclass(frozen=True)
class Options:
    delta: float = 0.35
    alpha: float = 0.2
    dot_delta: float = 1.0
    ddot_delta: float = 0.4
    lambda_q: float = 1.0
    lambda_s: float = 0.4
    lambda_l: float = 0.2
    epsilon: float = 0.1
    step_max: int = 8


@dataclass(frozen=True)
class Geometry:
    x: tuple[float, ...]
    y: tuple[float, ...]
    nx: tuple[float, ...]
    ny: tuple[float, ...]
    edge_lengths: tuple[float, ...]
    area: float
    perimeter: float
    c_a: tuple[float, ...]
    c_n: tuple[float, ...]


@dataclass(frozen=True)
class Result:
    objective: float
    max_constraint_violation: float
    normalized_area: float
    normalized_perimeter: float
    geometric_q: float
    scp_iterations: int
    offsets: tuple[float, ...]


def build_geometry(options: Options, fixture: str) -> Geometry:
    x, y = GEOMETRY_FIXTURES[fixture]
    count = len(x)
    nx: list[float] = []
    ny: list[float] = []
    edge_lengths: list[float] = []
    twice_area = 0.0
    for i in range(count):
        previous = (i - 1) % count
        following = (i + 1) % count
        tangent_x = x[following] - x[previous]
        tangent_y = y[following] - y[previous]
        tangent_length = math.hypot(tangent_x, tangent_y)
        nx.append(-tangent_y / tangent_length)
        ny.append(tangent_x / tangent_length)
        edge_lengths.append(math.hypot(x[following] - x[i], y[following] - y[i]))
        twice_area += x[i] * y[following] - y[i] * x[following]
    area = 0.5 * twice_area
    perimeter = sum(edge_lengths)
    c_a: list[float] = []
    c_n: list[float] = []
    for i in range(count):
        previous = (i - 1) % count
        following = (i + 1) % count
        c_a.append(
            0.5
            * (
                (y[following] - y[previous]) * nx[i]
                - (x[following] - x[previous]) * ny[i]
            )
            * options.delta
            / area
        )
        c_n.append(
            0.5
            * (ny[following] * nx[i] - nx[following] * ny[i])
            * options.delta**2
            / area
        )
    return Geometry(
        x=x,
        y=y,
        nx=tuple(nx),
        ny=tuple(ny),
        edge_lengths=tuple(edge_lengths),
        area=area,
        perimeter=perimeter,
        c_a=tuple(c_a),
        c_n=tuple(c_n),
    )


def active_terms(topology: str) -> tuple[bool, bool, bool]:
    return "q" in topology, "s" in topology, "l" in topology


def area_term(
    coefficient: float,
    current: gp.Var,
    following: gp.Var,
    reference: float,
    reference_following: float,
    *,
    upper: bool,
) -> gp.QuadExpr:
    summed = current + following
    reference_sum = reference + reference_following
    convex_sum_form = coefficient >= 0.0 if upper else coefficient < 0.0
    if convex_sum_form:
        return (
            0.5
            * coefficient
            * (
                summed * summed
                - 2.0 * reference * current
                - 2.0 * reference_following * following
                + reference * reference
                + reference_following * reference_following
            )
        )
    return (
        0.5
        * coefficient
        * (
            2.0 * reference_sum * summed
            - current * current
            - following * following
            - reference_sum * reference_sum
        )
    )


def geometric_metrics(
    geometry: Geometry, options: Options, offsets: tuple[float, ...]
) -> tuple[float, float, float]:
    child_x = tuple(
        geometry.x[i] + geometry.nx[i] * options.delta * offsets[i]
        for i in range(len(offsets))
    )
    child_y = tuple(
        geometry.y[i] + geometry.ny[i] * options.delta * offsets[i]
        for i in range(len(offsets))
    )
    twice_area = 0.0
    perimeter = 0.0
    for i in range(len(offsets)):
        following = (i + 1) % len(offsets)
        twice_area += child_x[i] * child_y[following] - child_y[i] * child_x[following]
        perimeter += math.hypot(
            child_x[following] - child_x[i], child_y[following] - child_y[i]
        )
    area = 0.5 * twice_area
    quotient = perimeter**2 / (4.0 * math.pi * area)
    return area / geometry.area, perimeter / geometry.perimeter, quotient


def solve_gurobi_subproblem(
    geometry: Geometry,
    options: Options,
    topology: str,
    reference: tuple[float, ...],
) -> Result:
    optimize_q, optimize_s, optimize_l = active_terms(topology)
    count = len(reference)
    model = gp.Model("iqop-parity")
    model.Params.OutputFlag = 0
    model.Params.NonConvex = 0
    model.Params.FeasibilityTol = SOLVER_TOLERANCE
    model.Params.OptimalityTol = SOLVER_TOLERANCE
    model.Params.BarQCPConvTol = SOLVER_TOLERANCE
    offsets = model.addVars(count, lb=options.alpha, ub=1.0, name="deltas")

    for i in range(count):
        previous = (i - 1) % count
        following = (i + 1) % count
        edge = geometry.edge_lengths[i]
        previous_edge = geometry.edge_lengths[previous]
        first = options.delta * (
            -(edge**2) * offsets[previous]
            + (edge**2 - previous_edge**2) * offsets[i]
            + previous_edge**2 * offsets[following]
        )
        first_bound = options.dot_delta * previous_edge * edge * (previous_edge + edge)
        model.addConstr(first <= first_bound)
        model.addConstr(first >= -first_bound)
        second = options.delta * (
            edge * offsets[previous]
            - (previous_edge + edge) * offsets[i]
            + previous_edge * offsets[following]
        )
        second_bound = (
            0.5 * options.ddot_delta * previous_edge * edge * (previous_edge + edge)
        )
        model.addConstr(second <= second_bound)
        model.addConstr(second >= -second_bound)

    area_plus = None
    area_minus = None
    length = None
    quotient = None
    if optimize_s:
        upper_model = gp.QuadExpr(1.0)
        for i in range(count):
            following = (i + 1) % count
            upper_model += geometry.c_a[i] * offsets[i]
            upper_model += area_term(
                geometry.c_n[i],
                offsets[i],
                offsets[following],
                reference[i],
                reference[following],
                upper=True,
            )
        area_plus = model.addVar(lb=0.0, name="area_plus")
        model.addQConstr(area_plus >= upper_model)

    if optimize_q:
        lower_model = gp.QuadExpr(1.0)
        for i in range(count):
            following = (i + 1) % count
            lower_model += geometry.c_a[i] * offsets[i]
            lower_model += area_term(
                geometry.c_n[i],
                offsets[i],
                offsets[following],
                reference[i],
                reference[following],
                upper=False,
            )
        area_minus = model.addVar(lb=0.0, name="area_minus")
        model.addQConstr(area_minus <= lower_model)

    if optimize_q or optimize_l:
        edge_x = model.addVars(count, lb=0.0, name="edge_x")
        edge_y = model.addVars(count, lb=0.0, name="edge_y")
        edge_length = model.addVars(count, lb=0.0, name="edge_length")
        for i in range(count):
            following = (i + 1) % count
            signed_x = (
                geometry.x[i]
                + geometry.nx[i] * options.delta * offsets[i]
                - geometry.x[following]
                - geometry.nx[following] * options.delta * offsets[following]
            ) / geometry.perimeter
            signed_y = (
                geometry.y[i]
                + geometry.ny[i] * options.delta * offsets[i]
                - geometry.y[following]
                - geometry.ny[following] * options.delta * offsets[following]
            ) / geometry.perimeter
            model.addConstr(edge_x[i] >= signed_x)
            model.addConstr(edge_x[i] >= -signed_x)
            model.addConstr(edge_y[i] >= signed_y)
            model.addConstr(edge_y[i] >= -signed_y)
            model.addQConstr(
                edge_length[i] * edge_length[i]
                >= edge_x[i] * edge_x[i] + edge_y[i] * edge_y[i]
            )
        length = model.addVar(lb=0.0, name="length")
        model.addConstr(length >= gp.quicksum(edge_length[i] for i in range(count)))

    if optimize_q:
        assert area_minus is not None and length is not None
        quotient = model.addVar(lb=1.0, name="quotient")
        area_to_perimeter_squared = geometry.area / geometry.perimeter**2
        model.addQConstr(
            length * length
            <= 4.0 * math.pi * area_to_perimeter_squared * area_minus * quotient
        )

    objective = gp.LinExpr()
    if optimize_q:
        assert quotient is not None
        objective += options.lambda_q * quotient
    if optimize_s:
        assert area_plus is not None
        objective += options.lambda_s * area_plus
    if optimize_l:
        assert length is not None
        objective += options.lambda_l * length
    model.setObjective(objective, GRB.MINIMIZE)
    model.optimize()
    assert model.Status == GRB.OPTIMAL

    solved_offsets = tuple(offsets[i].X for i in range(count))
    normalized_area, normalized_perimeter, geometric_q = geometric_metrics(
        geometry, options, solved_offsets
    )
    return Result(
        objective=model.ObjVal,
        max_constraint_violation=model.ConstrVio,
        normalized_area=normalized_area,
        normalized_perimeter=normalized_perimeter,
        geometric_q=geometric_q,
        scp_iterations=1,
        offsets=solved_offsets,
    )


def solve_gurobi(topology: str, mode: str, fixture: str) -> Result:
    options = Options()
    geometry = build_geometry(options, fixture)
    reference = tuple(0.5 * (1.0 + options.alpha) for _ in geometry.x)
    final = solve_gurobi_subproblem(geometry, options, topology, reference)
    if mode == "fixed":
        return final
    optimize_q, optimize_s, _ = active_terms(topology)
    for iteration in range(options.step_max):
        final = solve_gurobi_subproblem(geometry, options, topology, reference)
        physical_change = options.delta * max(
            abs(final.offsets[i] - reference[i]) for i in range(len(reference))
        )
        reference = final.offsets
        if (not optimize_q and not optimize_s) or (
            iteration > 1 and physical_change < options.epsilon
        ):
            return Result(**{**final.__dict__, "scp_iterations": iteration + 1})
    return Result(**{**final.__dict__, "scp_iterations": options.step_max})


def solve_ipopt(topology: str, mode: str, fixture: str) -> Result:
    executable = (
        Path(__file__).resolve().parent.parent
        / "build"
        / "pixi"
        / "tests"
        / "iqop_ipopt_reference"
    )
    completed = subprocess.run(
        [executable, topology, mode, fixture],
        check=True,
        capture_output=True,
        text=True,
    )
    rows = list(csv.DictReader(completed.stdout.splitlines()))
    assert len(rows) == 1
    row = rows[0]
    offsets = tuple(
        float(value) for name, value in row.items() if name.startswith("offset_")
    )
    return Result(
        objective=float(row["objective"]),
        max_constraint_violation=float(row["max_constraint_violation"]),
        normalized_area=float(row["normalized_area"]),
        normalized_perimeter=float(row["normalized_perimeter"]),
        geometric_q=float(row["geometric_q"]),
        scp_iterations=int(row["scp_iterations"]),
        offsets=offsets,
    )


def relative_gap(left: float, right: float) -> float:
    return abs(left - right) / max(1.0, abs(right))


def assert_parity(topology: str, mode: str, fixture: str) -> None:
    ipopt = solve_ipopt(topology, mode, fixture)
    gurobi = solve_gurobi(topology, mode, fixture)

    assert len(ipopt.offsets) == len(gurobi.offsets)
    assert ipopt.max_constraint_violation <= FEASIBILITY_GATE
    assert gurobi.max_constraint_violation <= FEASIBILITY_GATE
    assert relative_gap(ipopt.objective, gurobi.objective) <= OBJECTIVE_RELATIVE_GATE
    assert (
        relative_gap(ipopt.normalized_area, gurobi.normalized_area)
        <= GEOMETRY_RELATIVE_GATE
    )
    assert (
        relative_gap(ipopt.normalized_perimeter, gurobi.normalized_perimeter)
        <= GEOMETRY_RELATIVE_GATE
    )
    assert relative_gap(ipopt.geometric_q, gurobi.geometric_q) <= GEOMETRY_RELATIVE_GATE
    assert ipopt.scp_iterations == gurobi.scp_iterations
    assert (
        max(
            abs(ipopt.offsets[i] - gurobi.offsets[i]) for i in range(len(ipopt.offsets))
        )
        <= OFFSET_INFINITY_GATE
    )


@pytest.mark.parametrize("topology", ["q", "s", "l", "qs", "ql", "sl", "qsl"])
@pytest.mark.parametrize("mode", ["fixed", "scp"])
def test_ifopt_matches_gurobi(topology: str, mode: str) -> None:
    assert_parity(topology, mode, "irregular")


@pytest.mark.parametrize("fixture", ["convex", "slender"])
@pytest.mark.parametrize("mode", ["fixed", "scp"])
def test_ifopt_matches_gurobi_across_geometry(fixture: str, mode: str) -> None:
    assert_parity("qsl", mode, fixture)
