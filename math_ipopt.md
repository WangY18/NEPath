# IQOP SCP formulation for ifopt and Ipopt

## Purpose

The Ipopt implementation must solve the same sequential convex programming
(SCP) subproblems as the Gurobi implementation. Solver parity means matching
the Gurobi subproblem objective and feasibility, then matching the geometric
fixed point. It does not mean minimizing the exact nonconvex quotient in one
Ipopt call.

For a closed parent path with vertices
\(p_i=(x_i,y_i)\), inward unit normals \(n_i=(n_i^x,n_i^y)\), and cyclic
indices, the child path is

\[
\tilde p_i(\delta_i)=p_i+n_i\delta_i.
\]

Let \(n\) be the vertex count, \(A_0\) the parent area, \(L_0\) the parent
perimeter, \(\delta\) the line width, and \(\alpha\delta\) the minimum offset.
The build must reject degenerate paths, non-positive \(A_0\) or \(L_0\),
zero-length parent edges, invalid bounds, and an objective with no active term.

## Variable sets

### Offset variables

\[
\boldsymbol\delta=(\delta_0,\ldots,\delta_{n-1}),\qquad
\alpha\delta\le\delta_i\le\delta.
\]

### Edge epigraph variables

\[
D_i^x\ge0,\qquad D_i^y\ge0,\qquad D_i^l\ge0.
\]

These variables are present when perimeter or quotient optimization is active.

### Scalar epigraph variables

\[
A_+\ge0,\qquad A_-\ge0,\qquad L\ge0,\qquad Q\ge1.
\]

Only variables consumed by active objective terms and their constraints are
included. Inactive, zero-cost auxiliary variables would introduce unnecessary
KKT degeneracy in Ipopt.

## Smoothness constraints

Let \(l_i=\|p_{i+1}-p_i\|_2\), \(i^-=(i-1)\bmod n\), and
\(i^+=(i+1)\bmod n\). Define

\[
s_i=-l_i^2\delta_{i^-}+(l_i^2-l_{i^-}^2)\delta_i
    +l_{i^-}^2\delta_{i^+},
\]

\[
r_i=l_i\delta_{i^-}-(l_{i^-}+l_i)\delta_i
    +l_{i^-}\delta_{i^+}.
\]

The four linear inequalities per vertex are

\[
-\dot\delta_{\max}l_{i^-}l_i(l_{i^-}+l_i)
\le s_i\le
\dot\delta_{\max}l_{i^-}l_i(l_{i^-}+l_i),
\]

\[
-\tfrac12\ddot\delta_{\max}l_{i^-}l_i(l_{i^-}+l_i)
\le r_i\le
\tfrac12\ddot\delta_{\max}l_{i^-}l_i(l_{i^-}+l_i).
\]

The set contains \(4n\) rows. Every row has exactly three nonzero derivatives
with respect to the cyclic offset variables.

## Edge decomposition and norm epigraph

Define signed child-edge components

\[
e_i^x=x_i+n_i^x\delta_i-x_{i+1}-n_{i+1}^x\delta_{i+1},
\]

\[
e_i^y=y_i+n_i^y\delta_i-y_{i+1}-n_{i+1}^y\delta_{i+1}.
\]

Four linear inequalities per edge form absolute-value epigraphs:

\[
D_i^x\ge e_i^x,\quad D_i^x\ge-e_i^x,\quad
D_i^y\ge e_i^y,\quad D_i^y\ge-e_i^y.
\]

The smooth squared second-order-cone representation is

\[
(D_i^l)^2-(D_i^x)^2-(D_i^y)^2\ge0.
\]

Together with nonnegative \(D_i^l\), this is equivalent to
\(D_i^l\ge\sqrt{(D_i^x)^2+(D_i^y)^2}\). The squared form avoids the
nondifferentiable square-root apex, although its gradient still vanishes at
the cone apex. Initial auxiliary values therefore use strict positive slack.

The perimeter epigraph is

\[
L-\sum_iD_i^l\ge0.
\]

When \(L\) or \(Q\) is minimized, the epigraph chain is tight at the optimum.

## Area DC models

The signed area of the offset polygon is

\[
A(\boldsymbol\delta)=A_0+c_A^T\boldsymbol\delta
  +\sum_i c_{N,i}\delta_i\delta_{i+1},
\]

where

\[
c_{A,i}=\tfrac12\left((y_{i+1}-y_{i-1})n_i^x
 -(x_{i+1}-x_{i-1})n_i^y\right),
\]

\[
c_{N,i}=\tfrac12\left(n_{i+1}^yn_i^x-n_{i+1}^xn_i^y\right).
\]

At SCP reference \(\bar\delta\), every bilinear term is decomposed using

\[
2uv=(u+v)^2-u^2-v^2.
\]

For \(c=c_{N,i}\ge0\), define

\[
q_i^-(\delta;\bar\delta)=\tfrac c2\left[
2(\bar\delta_i+\bar\delta_{i+1})(\delta_i+\delta_{i+1})
-\delta_i^2-\delta_{i+1}^2
-(\bar\delta_i+\bar\delta_{i+1})^2\right],
\]

\[
q_i^+(\delta;\bar\delta)=\tfrac c2\left[
(\delta_i+\delta_{i+1})^2
-2\bar\delta_i\delta_i-2\bar\delta_{i+1}\delta_{i+1}
+\bar\delta_i^2+\bar\delta_{i+1}^2\right].
\]

Here \(q_i^-\) is a concave lower model and \(q_i^+\) is a convex upper
model of \(c\delta_i\delta_{i+1}\). For \(c<0\), the same two bracketed
expressions exchange upper/lower roles after multiplication by \(c\). The
sign split must match the Gurobi implementation exactly.

The aggregate models are

\[
A^-(\delta;\bar\delta)=A_0+c_A^T\delta+\sum_iq_i^-,
\qquad
A^+(\delta;\bar\delta)=A_0+c_A^T\delta+\sum_iq_i^+.
\]

Their convex constraints are written as

\[
A_+-A^+(\delta;\bar\delta)\ge0,
\qquad
A^-(\delta;\bar\delta)-A_-\ge0.
\]

Both models equal the true area at \(\delta=\bar\delta\), with matching first
derivatives.

## Isoperimetric quotient epigraph

The quotient constraint is the rotated cone

\[
L^2\le4\pi A_-Q,qquad A_-\ge0,\quad Q\ge1.
\]

For Ipopt it is evaluated as the smooth inequality

\[
4\pi A_-Q-L^2\ge0.
\]

Its scalar function has an indefinite Hessian even though the sign-restricted
feasible set is convex. Ipopt therefore provides no conic global-optimality
certificate; parity is established empirically against Gurobi.

## Objective

The active linear cost terms are

\[
f=\lambda_L\frac{L}{L_0}+\lambda_QQ
  +\lambda_S\frac{A_+}{A_0}.
\]

All length and area quantities are nondimensionalized at the modelling
boundary so objective and constraint scaling does not depend on input units.

## Ipopt numerical contract

The desired KKT tolerance is (10^{-8}). With a limited-memory Hessian,
dual feasibility can plateau after the primal SOCP constraints are already
resolved, so acceptable termination is permitted at (10^{-4}) only after
five consecutive acceptable iterations. The acceptable primal-constraint
violation remains (10^{-7}). Gurobi objective and geometry parity are the
downstream correctness gates for an acceptable-level solution.

Every edge and scalar epigraph is initialized with a strict (10^{-4})
interior margin. Ipopt's default `bound_push=0.01` must not be used here:
moving each near-zero (D_i^x,D_i^y,D_i^l) by (10^{-2}) accumulates in
(L\ge\sum_iD_i^l), turning a feasible start into an (O(n))-infeasible
start. Both variable and slack bound pushes/fractions are therefore set to
(10^{-6}), below the explicitly constructed interior margin. On the
256-vertex regression contour, this reduced the Q+S solve from 56 to 25 Ipopt
iterations.

Each convex subproblem is capped at 250 Ipopt iterations and 120 seconds wall
time. A complete public planner call has a separate 600-second solver budget
shared by all contours and SCP iterations. Exceeding either cap is a named
solve failure; neither limit is multiplied by the requested SCP step count.

## SCP loop

1. Initialize every \(\bar\delta_i=\tfrac12(1+\alpha)\delta\).
2. Build active variable, constraint, and cost sets around \(\bar\delta\).
3. Seed primal epigraph variables from the induced geometry with strict slack.
4. Solve with analytical Jacobians and limited-memory Hessian approximation.
5. Set \(\Delta=\|\delta-\bar\delta\|_\infty\) and update
   \(\bar\delta\leftarrow\delta\).
6. Stop when the same condition as the Gurobi path is met; otherwise rebuild
   the area models and solve the next subproblem.

## Parity contract

For an identical SCP reference point, the Ipopt and Gurobi subproblems are
compared using:

- normalized maximum constraint violation;
- relative weighted-objective gap;
- true geometric \(Q=L(\tilde l)^2/(4\pi A(\tilde l))\);
- normalized perimeter and area;
- normalized offset-vector infinity distance.

The automated equation-level harness covers every non-empty combination of Q,
S, and L on an irregular six-vertex fixture in fixed-reference and SCP modes,
plus Q+S+L on convex and slender fixtures. It compares a Gurobi Python
reconstruction with the public C++ ifopt/Ipopt formulation; it is not an
end-to-end call through NEPath's Gurobi backend.

Production-path parity additionally requires the same SCP stopping rule, zero
self-intersections, and matching downstream underfill rate at identical raster
resolution. Runtime and iteration counts are reported but are not correctness
gates.
