#include "NonEquidistant.h"
#include "Basic.h"
#include "ContourParallel.h"
#include "Connector.h"

#ifdef IncludeGurobi
NonEquidistant::NonEquidistant(bool debug/*=false*/) : gurobi(GRBEnv(true)) {
	// environment of gurobi
	if (!debug) {
		gurobi.set(GRB_IntParam_OutputFlag, 0); // Without log
	}
	gurobi.start();
}

double* NonEquidistant::Optimize_QSL(path p, const NonEquidistantOptions& opts) {
	// constants
	double* deltas_pre = new double[p.length]();
	for (register int i = 0; i < p.length; ++i) {
		deltas_pre[i] = 0.5 * (1.0 + opts.alpha) * opts.delta;
	}

	double A0 = Curve::AreaCal(p.x, p.y, p.length);
	double L0 = Curve::TotalLength(p.x, p.y, p.length, true);

	double* nx = NULL;
	double* ny = NULL;
	Curve::Ndir(p.x, p.y, p.length, nx, ny); // inward normal vector

	double* xtilde_pre = NULL;
	double* ytilde_pre = NULL;
	double* dl = NULL;

	// begin iteration
	Curve::OffsetNaive(p.x, p.y, deltas_pre, p.length, xtilde_pre, ytilde_pre, nx, ny);
	double L_pre = Curve::TotalLength(xtilde_pre, ytilde_pre, p.length, true);
	double A_plus_pre = Curve::AreaCal(xtilde_pre, ytilde_pre, p.length);
	double Q_pre = L_pre * L_pre / (4.0 * pi * A_plus_pre);

	double* c_A = new double[p.length];
	double* c_N = new double[p.length];
	for (register int i = 0; i < p.length; ++i) {
		c_A[i] = 0.5 * ((p.y[(i + 1) % p.length] - p.y[(i + p.length - 1) % p.length]) * nx[i] - (p.x[(i + 1) % p.length] - p.x[(i + p.length - 1) % p.length]) * ny[i]);
		c_N[i] = 0.5 * (ny[(i + 1) % p.length] * nx[i] - nx[(i + 1) % p.length] * ny[i]);
	}

	double* ub_deltas = new double[p.length];
	for (register int i = 0; i < p.length; ++i) {
		ub_deltas[i] = opts.delta;
	}
	double* ub_inf = new double[p.length];
	for (register int i = 0; i < p.length; ++i) {
		ub_inf[i] = GRB_INFINITY;
	}
	double* one_coeffs = new double[p.length];
	double* alpha_coeffs = new double[p.length];
	for (register int i = 0; i < p.length; ++i) {
		one_coeffs[i] = 1.0;
		alpha_coeffs[i] = opts.alpha * opts.delta;
	}

	// variables
	for (int i_iter = 0; i_iter < opts.step_max; ++i_iter) {
		GRBModel model = GRBModel(gurobi);
		GRBVar* deltas = model.addVars(alpha_coeffs, ub_deltas, NULL, NULL, NULL, NULL, p.length);
		Curve::DiffLength(dl, p.x, p.y, p.length, true);
		for (register int i = 0; i < p.length; ++i) {
			model.addConstr(-dl[i] * dl[i] * deltas[(i + p.length - 1) % p.length]
				+ (dl[i] * dl[i] - dl[(i + p.length - 1) % p.length] * dl[(i + p.length - 1) % p.length]) * deltas[i]
				+ dl[(i + p.length - 1) % p.length] * dl[(i + p.length - 1) % p.length] * deltas[(i + 1) % p.length]
				<= opts.dot_delta * dl[(i + p.length - 1) % p.length] * dl[i] * (dl[(i + p.length - 1) % p.length] + dl[i]));
			model.addConstr(-dl[i] * dl[i] * deltas[(i + p.length - 1) % p.length]
				+ (dl[i] * dl[i] - dl[(i + p.length - 1) % p.length] * dl[(i + p.length - 1) % p.length]) * deltas[i]
				+ dl[(i + p.length - 1) % p.length] * dl[(i + p.length - 1) % p.length] * deltas[(i + 1) % p.length]
				>= -opts.dot_delta * dl[(i + p.length - 1) % p.length] * dl[i] * (dl[(i + p.length - 1) % p.length] + dl[i]));
			model.addConstr(dl[i] * deltas[(i + p.length - 1) % p.length]
				- (dl[(i + p.length - 1) % p.length] + dl[i]) * deltas[i]
				+ dl[(i + p.length - 1) % p.length] * deltas[(i + 1) % p.length]
				<= 0.5 * opts.ddot_delta * dl[(i + p.length - 1) % p.length] * dl[i] * (dl[(i + p.length - 1) % p.length] + dl[i]));
			model.addConstr(dl[i] * deltas[(i + p.length - 1) % p.length]
				- (dl[(i + p.length - 1) % p.length] + dl[i]) * deltas[i]
				+ dl[(i + p.length - 1) % p.length] * deltas[(i + 1) % p.length]
				>= -0.5 * opts.ddot_delta * dl[(i + p.length - 1) % p.length] * dl[i] * (dl[(i + p.length - 1) % p.length] + dl[i]));
		}

		GRBQuadExpr A_plus_linear = A0;
		GRBQuadExpr A_minus_linear = A0;
		for (register int i = 0; i < p.length; ++i) {
			if (c_N[i] >= 0) {
				if (opts.optimize_Q) {
					A_minus_linear += 0.5 * c_N[i] * (2.0 * (
						deltas_pre[i] + deltas_pre[(i + 1) % p.length]) * (deltas[i] + deltas[(i + 1) % p.length])
						- deltas[i] * deltas[i] - deltas[(i + 1) % p.length] * deltas[(i + 1) % p.length]
						- (deltas_pre[i] + deltas_pre[(i + 1) % p.length]) * (deltas_pre[i] + deltas_pre[(i + 1) % p.length])); // q_minus
				}
				if (opts.optimize_S) {
					A_plus_linear += 0.5 * c_N[i] * (
						(deltas[i] + deltas[(i + 1) % p.length]) * (deltas[i] + deltas[(i + 1) % p.length])
						- 2.0 * deltas_pre[i] * deltas[i] - 2.0 * deltas_pre[(i + 1) % p.length] * deltas[(i + 1) % p.length]
						+ deltas_pre[i] * deltas_pre[i] + deltas_pre[(i + 1) % p.length] * deltas_pre[(i + 1) % p.length]); // q_plus
				}
			}
			else {
				if (opts.optimize_Q) {
					A_minus_linear += 0.5 * c_N[i] * (
						(deltas[i] + deltas[(i + 1) % p.length]) * (deltas[i] + deltas[(i + 1) % p.length])
						- 2.0 * deltas_pre[i] * deltas[i] - 2.0 * deltas_pre[(i + 1) % p.length] * deltas[(i + 1) % p.length]
						+ deltas_pre[i] * deltas_pre[i] + deltas_pre[(i + 1) % p.length] * deltas_pre[(i + 1) % p.length]); // q_plus
				}
				if (opts.optimize_S) {
					A_plus_linear += 0.5 * c_N[i] * (2.0 * (
						deltas_pre[i] + deltas_pre[(i + 1) % p.length]) * (deltas[i] + deltas[(i + 1) % p.length])
						- deltas[i] * deltas[i] - deltas[(i + 1) % p.length] * deltas[(i + 1) % p.length]
						- (deltas_pre[i] + deltas_pre[(i + 1) % p.length]) * (deltas_pre[i] + deltas_pre[(i + 1) % p.length])); // q_minus
				}
			}
		}
		GRBVar A_plus = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		GRBVar A_minus = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		if (opts.optimize_S) {
			A_plus_linear.addTerms(c_A, deltas, p.length);
			model.addQConstr(A_plus >= A_plus_linear);
		}
		if (opts.optimize_Q) {
			A_minus_linear.addTerms(c_A, deltas, p.length);
			model.addQConstr(A_minus <= A_minus_linear);
		}

		GRBVar* Dx = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, p.length);
		GRBVar* Dy = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, p.length);
		GRBVar* Dl = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, p.length);
		for (register int i = 0; i < p.length; ++i) {
			model.addConstr(Dx[i] >= p.x[i] + nx[i] * deltas[i] - p.x[(i + 1) % p.length] - nx[(i + 1) % p.length] * deltas[(i + 1) % p.length]);
			model.addConstr(Dx[i] >= -p.x[i] - nx[i] * deltas[i] + p.x[(i + 1) % p.length] + nx[(i + 1) % p.length] * deltas[(i + 1) % p.length]);
			model.addConstr(Dy[i] >= p.y[i] + ny[i] * deltas[i] - p.y[(i + 1) % p.length] - ny[(i + 1) % p.length] * deltas[(i + 1) % p.length]);
			model.addConstr(Dy[i] >= -p.y[i] - ny[i] * deltas[i] + p.y[(i + 1) % p.length] + ny[(i + 1) % p.length] * deltas[(i + 1) % p.length]);
			model.addQConstr(Dl[i] * Dl[i] >= Dx[i] * Dx[i] + Dy[i] * Dy[i]);
		}
		GRBLinExpr L_expr;
		L_expr.addTerms(one_coeffs, Dl, p.length);
		GRBVar L = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addConstr(L >= L_expr);

		GRBVar Q = model.addVar(1.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		if (opts.optimize_Q) {
			model.addQConstr(L * L + pi * A0 * (Q - A_minus / A0) * (Q - A_minus / A0) <= pi * A0 * (Q + A_minus / A0) * (Q + A_minus / A0));
		}


		GRBLinExpr obj = 0;
		if (opts.optimize_L) {
			obj += opts.lambda_L * L / L0;
		}
		if (opts.optimize_Q) {
			obj += opts.lambda_Q * Q;
		}
		if (opts.optimize_S) {
			obj += opts.lambda_S * A_plus / A0;
		}

		model.set(GRB_IntParam_NonConvex, 0);
		model.setObjective(obj, GRB_MINIMIZE);
		model.optimize();

		double Delta_delta = abs(deltas[0].get(GRB_DoubleAttr_X) - deltas_pre[0]);

		for (register int i = 0; i < p.length; ++i) {
			Delta_delta = max(Delta_delta, abs(deltas[i].get(GRB_DoubleAttr_X) - deltas_pre[i]));
			deltas_pre[i] = deltas[i].get(GRB_DoubleAttr_X);
		}
		if (opts.optimize_S) {
			A_plus_pre = A_plus.get(GRB_DoubleAttr_X);
		}
		if (opts.optimize_Q) {
			Q_pre = Q.get(GRB_DoubleAttr_X);
		}

		Curve::OffsetNaive(p.x, p.y, deltas_pre, p.length, xtilde_pre, ytilde_pre, nx, ny);
		double L_now = Curve::TotalLength(xtilde_pre, ytilde_pre, p.length, true);
		double A_now = Curve::AreaCal(xtilde_pre, ytilde_pre, p.length);
		double Q_now = L_now * L_now / (4 * pi * A_now);

		cout << "After " << i_iter + 1 << " iterations, Q is " << Q_now << ", A is " << A_now << ", L is " << L_now << ", and deltas varies " << Delta_delta << " now.\n";
		if ((!opts.optimize_Q && !opts.optimize_S) || ((i_iter > 1) && (Delta_delta < opts.epsilon)) || (i_iter > opts.step_max)) {
			break;
		}
	}

	delete[] ub_deltas;
	delete[] xtilde_pre;
	delete[] ytilde_pre;
	delete[] nx;
	delete[] ny;
	delete[] c_A;
	delete[] c_N;
	delete[] one_coeffs;
	delete[] dl;
	delete[] ub_inf;

	return deltas_pre;
}

pathnode* NonEquidistant::root_offset(const path& contour, const paths& holes, const NonEquidistantOptions& opts) {
	pathnode* root = new pathnode(contour);
	root->parent = NULL;
	std::stack<pathnode*> S;
	S.push(root);

	while (!S.empty()) {
		pathnode* pn_parent = S.top();
		S.pop();
		paths ps_offset = do1offset(pn_parent->data, opts);
		for (int i_offset = 0; i_offset < ps_offset.size(); ++i_offset) {
			paths ps_clip = ContourParallel::cut_holes(ps_offset[i_offset], holes, true, 0.1);
			for (int i_clip = 0; i_clip < ps_clip.size(); ++i_clip) {
				path p = opts.wash ? Curve::wash_dis(ps_clip[i_clip], opts.washdis) : ps_clip[i_clip];
				pathnode* pn_child = new pathnode(p);
				pn_parent->children.push_back(pn_child);
				pn_child->parent = pn_parent;
				S.push(pn_child);
			}
		}
	}
	return root;
}

paths NonEquidistant::NEpaths(const path& contour, const paths& holes, const NonEquidistantOptions& opts) {
	pathnode* root = root_offset(contour, holes, opts);
	ContourParallel::clearvoid(root, holes, opts.delta, pi * opts.delta * opts.delta * 0.05, 0.02);
	vector<pathnode*>* dfs = pathnode::DFS_root(root);
	paths solution;
	for (int i = 0; i < dfs->size(); ++i) {
		solution.push_back(path());
		solution[i].length = (*dfs)[i]->data.length;
		solution[i].x = (*dfs)[i]->data.x;
		solution[i].y = (*dfs)[i]->data.y;
		(*dfs)[i]->data.clear_without_delete();
		for (int j = 0; j < (*dfs)[i]->children.size(); ++j) {
			(*dfs)[i]->children[j] = NULL;
		}
	}
	delete dfs;
	delete root;
	return solution;
}

paths NonEquidistant::do1offset(const path& p, const NonEquidistantOptions& opts) {
	paths ps;
	double Length = Curve::TotalLength(p.x, p.y, p.length);
	paths ps1 = ContourParallel::OffsetClipper(p.x, p.y, opts.delta * 1.001, p.length, opts.wash, min(opts.washdis, Length * 1.0 / opts.num_least));
	if (ps1.size()) {
		for (int i_path = 0; i_path < ps1.size(); ++i_path) {
			Length = Curve::TotalLength(ps1[i_path].x, ps1[i_path].y, ps1[i_path].length);
			paths ps2 = ContourParallel::OffsetClipper(ps1[i_path].x, ps1[i_path].y, -opts.delta * 1.001, ps1[i_path].length, opts.wash, min(opts.washdis, Length * 1.0 / opts.num_least));
			for (int j = 0; j < ps2.size(); ++j) {
				double lambda = 0.1;
				double* deltas = Optimize_QSL(ps2[j], opts);
				if (deltas) {
					double max_deltas = deltas[0];
					for (register int i = 1; i < ps2[j].length; ++i) {
						max_deltas = (std::max)(max_deltas, deltas[i]);
					}
					for (register int i = 0; i < ps2[j].length; ++i) {
						deltas[i] += opts.delta - max_deltas;
					}
					ps.push_back(path());
					ps[ps.size() - 1].length = ps2[j].length;
					Curve::OffsetNaive(ps2[j].x, ps2[j].y, deltas, ps2[j].length, ps[ps.size() - 1].x, ps[ps.size() - 1].y);
					delete[] deltas;
				}
				else {
					paths ps3 = ContourParallel::OffsetClipper(ps2[j], paths(), opts.delta, opts.wash, opts.washdis, opts.num_least);
					ps.push_back(ps3[0]);
				}
			}
		}
	}
	return ps;
}

path NonEquidistant::NEpaths_CFS(const path& contour, const paths& holes, const NonEquidistantOptions& opts) {
	pathnode* root = root_offset(contour, holes, opts);
	ContourParallel::clearvoid(root, holes, opts.delta, pi * opts.delta * opts.delta * 0.09, 0.02);
	return Connector::ConnectedFermatSpiral_MultMinimum(root, opts.delta);
}

path NonEquidistant::NEpaths_DFS(const path& contour, const paths& holes, const NonEquidistantOptions& opts) {
	pathnode* root = root_offset(contour, holes, opts);
	path p = Connector::ConnectedDFS(root);
	delete root;
	return opts.wash ? p : Curve::wash_dis(p, opts.washdis);
}

#endif