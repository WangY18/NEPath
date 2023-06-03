#include "NonEquidistant.h"
#include "Basic.h"
#include "ContourParallel.h"

NonEquidistant::NonEquidistant(bool debug/*=false*/) : gurobi(GRBEnv(true)) {
	// environment of gurobi
	if (!debug) {
		gurobi.set(GRB_IntParam_OutputFlag, 0); // 不输出log信息
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
	double A_minus_pre = A_plus_pre;
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
		GRBVar* deltas = model.addVars(NULL, ub_deltas, NULL, NULL, NULL, NULL, p.length);
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
		// GRBVar L2 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		// model.addQConstr(L2 >= L * L);

		GRBVar Q = model.addVar(1.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		if (opts.optimize_Q) {
			model.addQConstr(L * L <= 2.0 * pi * Q_pre * A_minus_pre * (4.0 * (Q / Q_pre + A_minus / A_minus_pre) - Q * Q / (Q_pre * Q_pre) - A_minus * A_minus / (A_minus_pre * A_minus_pre) - 4.0));
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
			A_minus_pre = A_minus.get(GRB_DoubleAttr_X);
			Q_pre = Q.get(GRB_DoubleAttr_X);
		}

		Curve::OffsetNaive(p.x, p.y, deltas_pre, p.length, xtilde_pre, ytilde_pre, nx, ny);
		double L_now = Curve::TotalLength(xtilde_pre, ytilde_pre, p.length, true);
		double A_now = Curve::AreaCal(xtilde_pre, ytilde_pre, p.length);
		double Q_now = L_now * L_now / (4 * pi * A_now);

		cout << "After " << i_iter + 1 << " iterations, Q is " << Q_now << ", A is " << A_now << ", L is " << L_now << ", and deltas varies " << Delta_delta << " now.\n";
		if (((i_iter > 1) && (Delta_delta < opts.epsilon)) || (i_iter > opts.step_max)) {
			/*
			static int num = 0;
			++num;
			if (num == 10) {
				cout << "Here!\n";
			}*/
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

double* NonEquidistant::L_optimize(double const* x, double const* y, int length, double delta, double dot_delta, double ddot_delta, double alpha, double* delta0/*=NULL*/, double err/*=0.1*/, int N_maxiter/*=5*/) {
	// constants
	double* deltas_pre = new double[length]();
	if (delta0) {
		for (register int i = 0; i < length; ++i) {
			deltas_pre[i] = delta0[i];
		}
	}
	else {
		for (register int i = 0; i < length; ++i) {
			deltas_pre[i] = (1 + alpha) * 0.5 * delta;
		}
	}
	double* nx = NULL;
	double* ny = NULL;
	Curve::Ndir(x, y, length, nx, ny);
	double A0 = Curve::AreaCal(x, y, length);
	double L0 = Curve::TotalLength(x, y, length, true);

	double* xtilde_pre = NULL;
	double* ytilde_pre = NULL;
	double* dl = NULL;

	// begin iteration
	Curve::OffsetNaive(x, y, deltas_pre, length, xtilde_pre, ytilde_pre, nx, ny);

	double* lb_deltas = new double[length];
	double* ub_deltas = new double[length];
	for (register int i = 0; i < length; ++i) {
		lb_deltas[i] = alpha * delta;
		ub_deltas[i] = delta;
	}
	double* ub_inf = new double[length];
	for (register int i = 0; i < length; ++i) {
		ub_inf[i] = GRB_INFINITY;
	}
	double* one_coeffs = new double[length];
	for (register int i = 0; i < length; ++i) {
		one_coeffs[i] = 1.0;
	}

	// variables
	GRBModel model = GRBModel(gurobi);
	GRBVar* deltas = model.addVars(lb_deltas, ub_deltas, NULL, NULL, NULL, NULL, length);
	Curve::DiffLength(dl, x, y, length, true);
	for (register int i = 0; i < length; ++i) { // Warning for adding constraints: zero or small (< 1e-13) coefficients, ignored
		model.addConstr(-dl[i] * dl[i] * deltas[(i + length - 1) % length]
			+ (dl[i] * dl[i] - dl[(i + length - 1) % length] * dl[(i + length - 1) % length]) * deltas[i]
			+ dl[(i + length - 1) % length] * dl[(i + length - 1) % length] * deltas[(i + 1) % length]
			<= dot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
		model.addConstr(-dl[i] * dl[i] * deltas[(i + length - 1) % length]
			+ (dl[i] * dl[i] - dl[(i + length - 1) % length] * dl[(i + length - 1) % length]) * deltas[i]
			+ dl[(i + length - 1) % length] * dl[(i + length - 1) % length] * deltas[(i + 1) % length]
			>= -dot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
		model.addConstr(dl[i] * deltas[(i + length - 1) % length]
			- (dl[(i + length - 1) % length] + dl[i]) * deltas[i]
			+ dl[(i + length - 1) % length] * deltas[(i + 1) % length]
			<= 0.5 * ddot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
		model.addConstr(dl[i] * deltas[(i + length - 1) % length]
			- (dl[(i + length - 1) % length] + dl[i]) * deltas[i]
			+ dl[(i + length - 1) % length] * deltas[(i + 1) % length]
			>= -0.5 * ddot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
	}

	GRBVar* Dx = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, length);
	GRBVar* Dy = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, length);
	GRBVar* Dl = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, length);
	for (register int i = 0; i < length; ++i) {
		model.addConstr(Dx[i] >= x[i] + nx[i] * deltas[i] - x[(i + 1) % length] - nx[(i + 1) % length] * deltas[(i + 1) % length]);
		model.addConstr(Dx[i] >= -x[i] - nx[i] * deltas[i] + x[(i + 1) % length] + nx[(i + 1) % length] * deltas[(i + 1) % length]);
		model.addConstr(Dy[i] >= y[i] + ny[i] * deltas[i] - y[(i + 1) % length] - ny[(i + 1) % length] * deltas[(i + 1) % length]);
		model.addConstr(Dy[i] >= -y[i] - ny[i] * deltas[i] + y[(i + 1) % length] + ny[(i + 1) % length] * deltas[(i + 1) % length]);
		model.addQConstr(Dl[i] * Dl[i] >= Dx[i] * Dx[i] + Dy[i] * Dy[i]);
	}
	GRBLinExpr L_expr;
	L_expr.addTerms(one_coeffs, Dl, length);
	GRBVar L = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
	model.addConstr(L >= L_expr);


	GRBLinExpr obj = L;
	model.setObjective(obj, GRB_MINIMIZE);
	model.optimize();
	for (register int i = 0; i < length; ++i) {
		deltas_pre[i] = deltas[i].get(GRB_DoubleAttr_X);
	}

	Curve::OffsetNaive(x, y, deltas_pre, length, xtilde_pre, ytilde_pre, nx, ny);


	double Lnow = L.get(GRB_DoubleAttr_X);
	cout << "Length of path is " << Lnow << " now.\n";

	delete[] lb_deltas;
	delete[] ub_deltas;
	delete[] xtilde_pre;
	delete[] ytilde_pre;
	delete[] nx;
	delete[] ny;
	delete[] one_coeffs;
	delete[] dl;
	delete[] ub_inf;
	return deltas_pre;
}

double* NonEquidistant::IQA_optimize(double const* x, double const* y, int length, double delta, double dot_delta, double ddot_delta, double alpha, double lambda, double* delta0/*=NULL*/, double err/*=0.1*/, int N_maxiter/*=5*/) {
	// 圆度+面积优化

	// constants
	double* deltas_pre = new double[length]();
	if (delta0) {
		for (register int i = 0; i < length; ++i) {
			deltas_pre[i] = delta0[i];
		}
	}
	else {
		for (register int i = 0; i < length; ++i) {
			deltas_pre[i] = (1 + alpha) * 0.5 * delta;
		}
	}
	double* nx = NULL;
	double* ny = NULL;
	Curve::Ndir(x, y, length, nx, ny);
	double A0 = Curve::AreaCal(x, y, length);
	double L0 = Curve::TotalLength(x, y, length, true);

	double* xtilde_pre = NULL;
	double* ytilde_pre = NULL;
	double* dl = NULL;

	// begin iteration
	Curve::OffsetNaive(x, y, deltas_pre, length, xtilde_pre, ytilde_pre, nx, ny);
	double L_pre = Curve::TotalLength(xtilde_pre, ytilde_pre, length, true);
	double A_plus_pre = Curve::AreaCal(xtilde_pre, ytilde_pre, length);
	double A_minus_pre = A_plus_pre;
	double T_pre = L_pre * L_pre / (4.0 * pi * A_plus_pre);

	double* c_A = new double[length];
	double* c_N = new double[length];
	for (register int i = 0; i < length; ++i) {
		c_A[i] = 0.5 * ((y[(i + 1) % length] - y[(i + length - 1) % length]) * nx[i] - (x[(i + 1) % length] - x[(i + length - 1) % length]) * ny[i]);
		c_N[i] = 0.5 * (ny[(i + 1) % length] * nx[i] - nx[(i + 1) % length] * ny[i]);
	}

	double* lb_deltas = new double[length];
	double* ub_deltas = new double[length];
	for (register int i = 0; i < length; ++i) {
		lb_deltas[i] = alpha * delta;
		ub_deltas[i] = delta;
	}
	double* ub_inf = new double[length];
	for (register int i = 0; i < length; ++i) {
		ub_inf[i] = GRB_INFINITY;
	}
	double* one_coeffs = new double[length];
	for (register int i = 0; i < length; ++i) {
		one_coeffs[i] = 1.0;
	}

	double* T_all = new double[N_maxiter + 1];
	T_all[0] = T_pre +lambda * A_plus_pre / A0;// +(0.01 / L0) * L_pre;

	// variables
	for (int i_iter = 0; i_iter < N_maxiter; ++i_iter) {
		GRBModel model = GRBModel(gurobi);
		GRBVar* deltas = model.addVars(lb_deltas, ub_deltas, NULL, NULL, NULL, NULL, length);
		// addcons_ddot_deltas(model, x, y, deltas, length, dot_delta, ddot_delta);
		Curve::DiffLength(dl, x, y, length, true);
		for (register int i = 0; i < length; ++i) { // Warning for adding constraints: zero or small (< 1e-13) coefficients, ignored
			model.addConstr(-dl[i] * dl[i] * deltas[(i + length - 1) % length]
				+ (dl[i] * dl[i] - dl[(i + length - 1) % length] * dl[(i + length - 1) % length]) * deltas[i]
				+ dl[(i + length - 1) % length] * dl[(i + length - 1) % length] * deltas[(i + 1) % length]
				<= dot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
			model.addConstr(-dl[i] * dl[i] * deltas[(i + length - 1) % length]
				+ (dl[i] * dl[i] - dl[(i + length - 1) % length] * dl[(i + length - 1) % length]) * deltas[i]
				+ dl[(i + length - 1) % length] * dl[(i + length - 1) % length] * deltas[(i + 1) % length]
				>= -dot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
			model.addConstr(dl[i] * deltas[(i + length - 1) % length]
				- (dl[(i + length - 1) % length] + dl[i]) * deltas[i]
				+ dl[(i + length - 1) % length] * deltas[(i + 1) % length]
				<= 0.5 * ddot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
			model.addConstr(dl[i] * deltas[(i + length - 1) % length]
				- (dl[(i + length - 1) % length] + dl[i]) * deltas[i]
				+ dl[(i + length - 1) % length] * deltas[(i + 1) % length]
				>= -0.5 * ddot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
		}

		GRBQuadExpr A_plus_linear = A0;
		GRBQuadExpr A_minus_linear = A0;
		for (register int i = 0; i < length; ++i) {
			if (c_N[i] >= 0) {
				A_minus_linear += 0.5 * c_N[i] * (2.0 * (
					deltas_pre[i] + deltas_pre[(i + 1) % length]) * (deltas[i] + deltas[(i + 1) % length])
					- deltas[i] * deltas[i] - deltas[(i + 1) % length] * deltas[(i + 1) % length]
					- (deltas_pre[i] + deltas_pre[(i + 1) % length]) * (deltas_pre[i] + deltas_pre[(i + 1) % length])); // q_minus
				A_plus_linear += 0.5 * c_N[i] * (
					(deltas[i] + deltas[(i + 1) % length]) * (deltas[i] + deltas[(i + 1) % length])
					- 2.0 * deltas_pre[i] * deltas[i] - 2.0 * deltas_pre[(i + 1) % length] * deltas[(i + 1) % length]
					+ deltas_pre[i] * deltas_pre[i] + deltas_pre[(i + 1) % length] * deltas_pre[(i + 1) % length]); // q_plus
			}
			else {
				A_minus_linear += 0.5 * c_N[i] * (
					(deltas[i] + deltas[(i + 1) % length]) * (deltas[i] + deltas[(i + 1) % length])
					- 2.0 * deltas_pre[i] * deltas[i] - 2.0 * deltas_pre[(i + 1) % length] * deltas[(i + 1) % length]
					+ deltas_pre[i] * deltas_pre[i] + deltas_pre[(i + 1) % length] * deltas_pre[(i + 1) % length]); // q_plus
				A_plus_linear += 0.5 * c_N[i] * (2.0 * (
					deltas_pre[i] + deltas_pre[(i + 1) % length]) * (deltas[i] + deltas[(i + 1) % length])
					- deltas[i] * deltas[i] - deltas[(i + 1) % length] * deltas[(i + 1) % length]
					- (deltas_pre[i] + deltas_pre[(i + 1) % length]) * (deltas_pre[i] + deltas_pre[(i + 1) % length])); // q_minus
			}
		}
		A_plus_linear.addTerms(c_A, deltas, length);
		A_minus_linear.addTerms(c_A, deltas, length);
		GRBVar A_plus = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		GRBVar A_minus = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addQConstr(A_plus >= A_plus_linear);
		model.addQConstr(A_minus <= A_minus_linear);

		GRBVar* Dx = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, length);
		GRBVar* Dy = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, length);
		GRBVar* Dl = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, length);
		for (register int i = 0; i < length; ++i) {
			model.addConstr(Dx[i] >= x[i] + nx[i] * deltas[i] - x[(i + 1) % length] - nx[(i + 1) % length] * deltas[(i + 1) % length]);
			model.addConstr(Dx[i] >= -x[i] - nx[i] * deltas[i] + x[(i + 1) % length] + nx[(i + 1) % length] * deltas[(i + 1) % length]);
			model.addConstr(Dy[i] >= y[i] + ny[i] * deltas[i] - y[(i + 1) % length] - ny[(i + 1) % length] * deltas[(i + 1) % length]);
			model.addConstr(Dy[i] >= -y[i] - ny[i] * deltas[i] + y[(i + 1) % length] + ny[(i + 1) % length] * deltas[(i + 1) % length]);
			model.addQConstr(Dl[i] * Dl[i] >= Dx[i] * Dx[i] + Dy[i] * Dy[i]);
		}
		GRBLinExpr L_expr;
		L_expr.addTerms(one_coeffs, Dl, length);
		GRBVar L = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addConstr(L >= L_expr);
		GRBVar L2 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addQConstr(L2 >= L * L);

		GRBVar T = model.addVar(1.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addQConstr(L2 <= 2.0 * pi * T_pre * A_minus_pre * (4.0 * (T / T_pre + A_minus / A_minus_pre) - T * T / (T_pre * T_pre) - A_minus * A_minus / (A_minus_pre * A_minus_pre) - 4.0));


		GRBLinExpr obj = T + (lambda / A0) * A_plus;//  +(0.01 / L0) * L; //
		//model.set(GRB_IntParam_NonConvex, 0);
		model.setObjective(obj, GRB_MINIMIZE);
		model.optimize();
		for (register int i = 0; i < length; ++i) {
			deltas_pre[i] = deltas[i].get(GRB_DoubleAttr_X);
		}
		// A_plus_pre = A_plus.get(GRB_DoubleAttr_X);
		A_minus_pre = A_minus.get(GRB_DoubleAttr_X);
		T_pre = T.get(GRB_DoubleAttr_X);

		Curve::OffsetNaive(x, y, deltas_pre, length, xtilde_pre, ytilde_pre, nx, ny);
		double L_now = Curve::TotalLength(xtilde_pre, ytilde_pre, length, true);
		double A_now = Curve::AreaCal(xtilde_pre, ytilde_pre, length);
		double T_now = L_now * L_now / (4 * pi * A_now);
		T_all[i_iter + 1] = T_pre +lambda * A_plus_pre / A0;
		if ((i_iter > 0) && (abs(T_all[i_iter + 1] - T_all[i_iter]) < abs(T_all[i_iter - 1] - T_all[i_iter]) * err)) {
			/*
			static int num = 0;
			++num;
			if (num == 10) {
				cout << "Here!\n";
			}*/
			cout << "After " << i_iter << " iterations, T is " << T_now << " now.\n";
			break;
		}
	}

	delete[] lb_deltas;
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
	delete[] T_all;
	return deltas_pre;
}

double* NonEquidistant::L2A_optimize(double const* x, double const* y, int length, double delta, double dot_delta, double ddot_delta, double alpha, double* delta0/*=NULL*/, double err/*=0.1*/, int N_maxiter/*=5*/) {
	// 纯圆度优化
	// 跟MATLAB计算结果略有不同
	// Warning: Quadratic constraints contain large coefficient range Consider reformulating model or setting NumericFocus parameter to avoid numerical issues.

	// constants
	double* deltas_pre = new double[length]();
	if (delta0) {
		for (register int i = 0; i < length; ++i) {
			deltas_pre[i] = delta0[i];
		}
	}
	else {
		for (register int i = 0; i < length; ++i) {
			deltas_pre[i] = (1 + alpha) * 0.5 * delta;
		}
	}
	double* nx = NULL;
	double* ny = NULL;
	Curve::Ndir(x, y, length, nx, ny);
	double A0 = Curve::AreaCal(x, y, length);
	double L0 = Curve::TotalLength(x, y, length, true);

	double* xtilde_pre = NULL;
	double* ytilde_pre = NULL;
	double* dl = NULL;

	// begin iteration
	Curve::OffsetNaive(x, y, deltas_pre, length, xtilde_pre, ytilde_pre, nx, ny);
	double L_pre = Curve::TotalLength(xtilde_pre, ytilde_pre, length, true);
	double A_pre = Curve::AreaCal(xtilde_pre, ytilde_pre, length);
	double T_pre = L_pre * L_pre / (4.0 * pi * A_pre);
	/*
	if (T_pre < 1 + 1e-3) {
		cout << "After " << 0 << " iterations, T is " << T_pre << " now.\n";
		delete[] xtilde_pre;
		delete[] ytilde_pre;
		delete[] deltas_pre;
		delete[] nx;
		delete[] ny;
		delete[] dl;
		return NULL;
	}*/

	double* c_A = new double[length];
	double* c_N = new double[length];
	for (register int i = 0; i < length; ++i) {
		c_A[i] = 0.5 * ((y[(i + 1) % length] - y[(i + length - 1) % length]) * nx[i] - (x[(i + 1) % length] - x[(i + length - 1) % length]) * ny[i]);
		c_N[i] = 0.5 * (ny[(i + 1) % length] * nx[i] - nx[(i + 1) % length] * ny[i]);
	}

	double* lb_deltas = new double[length];
	double* ub_deltas = new double[length];
	for (register int i = 0; i < length; ++i) {
		lb_deltas[i] = alpha * delta;
		ub_deltas[i] = delta;
	}
	double* ub_inf = new double[length];
	for (register int i = 0; i < length; ++i) {
		ub_inf[i] = GRB_INFINITY;
	}
	double* one_coeffs = new double[length];
	for (register int i = 0; i < length; ++i) {
		one_coeffs[i] = 1.0;
	}

	double lambda = 0.0;
	double* T_all = new double[N_maxiter + 1];
	T_all[0] = T_pre;// +lambda * A_pre / A0;// +(0.01 / L0) * L_pre;

	// variables
	for (int i_iter = 0; i_iter < N_maxiter; ++i_iter) {
		GRBModel model = GRBModel(gurobi);
		GRBVar* deltas = model.addVars(lb_deltas, ub_deltas, NULL, NULL, NULL, NULL, length);
		// addcons_ddot_deltas(model, x, y, deltas, length, dot_delta, ddot_delta);
		Curve::DiffLength(dl, x, y, length, true);
		for (register int i = 0; i < length; ++i) { // Warning for adding constraints: zero or small (< 1e-13) coefficients, ignored
			model.addConstr(-dl[i] * dl[i] * deltas[(i + length - 1) % length]
				+ (dl[i] * dl[i] - dl[(i + length - 1) % length] * dl[(i + length - 1) % length]) * deltas[i]
				+ dl[(i + length - 1) % length] * dl[(i + length - 1) % length] * deltas[(i + 1) % length]
				<= dot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
			model.addConstr(-dl[i] * dl[i] * deltas[(i + length - 1) % length]
				+ (dl[i] * dl[i] - dl[(i + length - 1) % length] * dl[(i + length - 1) % length]) * deltas[i]
				+ dl[(i + length - 1) % length] * dl[(i + length - 1) % length] * deltas[(i + 1) % length]
				>= -dot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
			model.addConstr(dl[i] * deltas[(i + length - 1) % length]
				- (dl[(i + length - 1) % length] + dl[i]) * deltas[i]
				+ dl[(i + length - 1) % length] * deltas[(i + 1) % length]
				<= 0.5 * ddot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
			model.addConstr(dl[i] * deltas[(i + length - 1) % length]
				- (dl[(i + length - 1) % length] + dl[i]) * deltas[i]
				+ dl[(i + length - 1) % length] * deltas[(i + 1) % length]
				>= -0.5 * ddot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
		}

		GRBQuadExpr A_linear = A0;
		for (register int i = 0; i < length; ++i) {
			if (c_N[i] >= 0) {
				A_linear += 0.5 * c_N[i] * (2.0 * (
					deltas_pre[i] + deltas_pre[(i + 1) % length]) * (deltas[i] + deltas[(i + 1) % length])
					- deltas[i] * deltas[i] - deltas[(i + 1) % length] * deltas[(i + 1) % length]
					- (deltas_pre[i] + deltas_pre[(i + 1) % length]) * (deltas_pre[i] + deltas_pre[(i + 1) % length]));
			}
			else {
				A_linear += 0.5 * c_N[i] * (
					(deltas[i] + deltas[(i + 1) % length]) * (deltas[i] + deltas[(i + 1) % length])
					- 2.0 * deltas_pre[i] * deltas[i] - 2.0 * deltas_pre[(i + 1) % length] * deltas[(i + 1) % length]
					+ deltas_pre[i] * deltas_pre[i] + deltas_pre[(i + 1) % length] * deltas_pre[(i + 1) % length]);
			}
		}
		A_linear.addTerms(c_A, deltas, length); // Warning: zero or small (< 1e-13) coefficients in quadratic constraints, ignored
		GRBVar A = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addQConstr(A <= A_linear);

		GRBVar* Dx = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, length);
		GRBVar* Dy = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, length);
		GRBVar* Dl = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, length);
		for (register int i = 0; i < length; ++i) {
			model.addConstr(Dx[i] >= x[i] + nx[i] * deltas[i] - x[(i + 1) % length] - nx[(i + 1) % length] * deltas[(i + 1) % length]);
			model.addConstr(Dx[i] >= -x[i] - nx[i] * deltas[i] + x[(i + 1) % length] + nx[(i + 1) % length] * deltas[(i + 1) % length]);
			model.addConstr(Dy[i] >= y[i] + ny[i] * deltas[i] - y[(i + 1) % length] - ny[(i + 1) % length] * deltas[(i + 1) % length]);
			model.addConstr(Dy[i] >= -y[i] - ny[i] * deltas[i] + y[(i + 1) % length] + ny[(i + 1) % length] * deltas[(i + 1) % length]);
			model.addQConstr(Dl[i] * Dl[i] >= Dx[i] * Dx[i] + Dy[i] * Dy[i]);
		}
		GRBLinExpr L_expr;
		L_expr.addTerms(one_coeffs, Dl, length);
		GRBVar L = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addConstr(L >= L_expr);
		GRBVar L2 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addQConstr(L2 >= L * L);

		GRBVar T = model.addVar(1.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addQConstr(L2 <= 2.0 * pi * T_pre * A_pre * (4.0 * (T / T_pre + A / A_pre) - T * T / (T_pre * T_pre) - A * A / (A_pre * A_pre) - 4.0));

		// set the initial value. It is unnecessary since the model is not MIP.
		/*
		for (register int i = 0; i < length; ++i) {
			deltas[i].set(GRB_DoubleAttr_Start, deltas_pre[i]);
			double Dx_double = x[i] + nx[i] * deltas_pre[i] - x[(i + 1) % length] - nx[(i + 1) % length] * deltas_pre[(i + 1) % length];
			double Dy_double = y[i] + ny[i] * deltas_pre[i] - y[(i + 1) % length] - ny[(i + 1) % length] * deltas_pre[(i + 1) % length];
			double Dl_double = sqrt(Dx_double * Dx_double + Dy_double * Dy_double);
			Dx[i].set(GRB_DoubleAttr_Start, abs(Dx_double));
			Dy[i].set(GRB_DoubleAttr_Start, abs(Dy_double));
			Dl[i].set(GRB_DoubleAttr_Start, Dl_double);
		}
		L.set(GRB_DoubleAttr_Start, L_pre);
		L2.set(GRB_DoubleAttr_Start, L_pre* L_pre);
		A.set(GRB_DoubleAttr_Start, A_pre);
		T.set(GRB_DoubleAttr_Start, T_pre);
		*/

		GRBLinExpr obj = T;// +(lambda / A0) * A;//  +(0.01 / L0) * L; //
		//model.set(GRB_IntParam_NonConvex, 0);
		model.setObjective(obj, GRB_MINIMIZE);
		model.optimize();
		for (register int i = 0; i < length; ++i) {
			deltas_pre[i] = deltas[i].get(GRB_DoubleAttr_X);
		}
		A_pre = A.get(GRB_DoubleAttr_X);
		T_pre = T.get(GRB_DoubleAttr_X);

		Curve::OffsetNaive(x, y, deltas_pre, length, xtilde_pre, ytilde_pre, nx, ny);
		double L_now = Curve::TotalLength(xtilde_pre, ytilde_pre, length, true);
		double A_now = Curve::AreaCal(xtilde_pre, ytilde_pre, length);
		double T_now = L_now * L_now / (4 * pi * A_now);
		// T_pre = T_now;
		// A_pre = A_now;
		T_all[i_iter + 1] = T_pre;// +lambda * A_pre / A0;// +(0.01 / L0) * L.get(GRB_DoubleAttr_X);
		//T_all[i_iter + 1] = T_now + lambda * A_now / A0;
		err = 0.1;
		//if (false) {
		if ((T_now < 1.0001) || ((i_iter > 0) && (abs(T_all[i_iter + 1] - T_all[i_iter]) < abs(T_all[i_iter - 1] - T_all[i_iter]) * err))) {
			cout << "After " << i_iter << " iterations, T is " << T_now << " now.\n";
			break;
		}
	}
	
	delete[] lb_deltas;
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
	delete[] T_all;
	return deltas_pre;
}

double* NonEquidistant::QSL_optimize_outward(double const* x, double const* y, int length, double delta, double dot_delta, double ddot_delta, double lambdaQ, double lambdaS, double lambdaL, double err/*=0.1*/, int N_maxiter/*=5*/) {
	// 边界平滑用的优化问题，圆度、面积和长度
	// constants
	double* deltas_pre = new double[length]();
	for (register int i = 0; i < length; ++i) {
		deltas_pre[i] = 0.5 * delta;
	}

	double* nx = NULL;
	double* ny = NULL;
	Curve::Ndir(x, y, length, nx, ny); // 内法向
	for (int i = 0; i < length; ++i) { // 调整为外法向
		nx[i] *= -1;
		ny[i] *= -1;
	}
	double A0 = Curve::AreaCal(x, y, length);
	double L0 = Curve::TotalLength(x, y, length, true);

	double* xtilde_pre = NULL;
	double* ytilde_pre = NULL;
	double* dl = NULL;

	// begin iteration
	Curve::OffsetNaive(x, y, deltas_pre, length, xtilde_pre, ytilde_pre, nx, ny);
	double L_pre = Curve::TotalLength(xtilde_pre, ytilde_pre, length, true);
	double A_plus_pre = Curve::AreaCal(xtilde_pre, ytilde_pre, length);
	double A_minus_pre = A_plus_pre;
	double T_pre = L_pre * L_pre / (4.0 * pi * A_plus_pre);

	double* c_A = new double[length];
	double* c_N = new double[length];
	for (register int i = 0; i < length; ++i) {
		c_A[i] = 0.5 * ((y[(i + 1) % length] - y[(i + length - 1) % length]) * nx[i] - (x[(i + 1) % length] - x[(i + length - 1) % length]) * ny[i]);
		c_N[i] = 0.5 * (ny[(i + 1) % length] * nx[i] - nx[(i + 1) % length] * ny[i]);
	}

	double* ub_deltas = new double[length];
	for (register int i = 0; i < length; ++i) {
		ub_deltas[i] = delta;
	}
	double* ub_inf = new double[length];
	for (register int i = 0; i < length; ++i) {
		ub_inf[i] = GRB_INFINITY;
	}
	double* one_coeffs = new double[length];
	for (register int i = 0; i < length; ++i) {
		one_coeffs[i] = 1.0;
	}

	double* T_all = new double[N_maxiter + 1];
	T_all[0] = lambdaQ * T_pre + lambdaS * A_plus_pre / A0 + lambdaL * L_pre / L0;

	// variables
	for (int i_iter = 0; i_iter < N_maxiter; ++i_iter) {
		GRBModel model = GRBModel(gurobi);
		GRBVar* deltas = model.addVars(NULL, ub_deltas, NULL, NULL, NULL, NULL, length);
		// addcons_ddot_deltas(model, x, y, deltas, length, dot_delta, ddot_delta);
		Curve::DiffLength(dl, x, y, length, true);
		for (register int i = 0; i < length; ++i) { // Warning for adding constraints: zero or small (< 1e-13) coefficients, ignored
			model.addConstr(-dl[i] * dl[i] * deltas[(i + length - 1) % length]
				+ (dl[i] * dl[i] - dl[(i + length - 1) % length] * dl[(i + length - 1) % length]) * deltas[i]
				+ dl[(i + length - 1) % length] * dl[(i + length - 1) % length] * deltas[(i + 1) % length]
				<= dot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
			model.addConstr(-dl[i] * dl[i] * deltas[(i + length - 1) % length]
				+ (dl[i] * dl[i] - dl[(i + length - 1) % length] * dl[(i + length - 1) % length]) * deltas[i]
				+ dl[(i + length - 1) % length] * dl[(i + length - 1) % length] * deltas[(i + 1) % length]
				>= -dot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
			model.addConstr(dl[i] * deltas[(i + length - 1) % length]
				- (dl[(i + length - 1) % length] + dl[i]) * deltas[i]
				+ dl[(i + length - 1) % length] * deltas[(i + 1) % length]
				<= 0.5 * ddot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
			model.addConstr(dl[i] * deltas[(i + length - 1) % length]
				- (dl[(i + length - 1) % length] + dl[i]) * deltas[i]
				+ dl[(i + length - 1) % length] * deltas[(i + 1) % length]
				>= -0.5 * ddot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
		}

		GRBQuadExpr A_plus_linear = A0;
		GRBQuadExpr A_minus_linear = A0;
		for (register int i = 0; i < length; ++i) {
			if (c_N[i] >= 0) {
				A_minus_linear += 0.5 * c_N[i] * (2.0 * (
					deltas_pre[i] + deltas_pre[(i + 1) % length]) * (deltas[i] + deltas[(i + 1) % length])
					- deltas[i] * deltas[i] - deltas[(i + 1) % length] * deltas[(i + 1) % length]
					- (deltas_pre[i] + deltas_pre[(i + 1) % length]) * (deltas_pre[i] + deltas_pre[(i + 1) % length])); // q_minus
				A_plus_linear += 0.5 * c_N[i] * (
					(deltas[i] + deltas[(i + 1) % length]) * (deltas[i] + deltas[(i + 1) % length])
					- 2.0 * deltas_pre[i] * deltas[i] - 2.0 * deltas_pre[(i + 1) % length] * deltas[(i + 1) % length]
					+ deltas_pre[i] * deltas_pre[i] + deltas_pre[(i + 1) % length] * deltas_pre[(i + 1) % length]); // q_plus
			}
			else {
				A_minus_linear += 0.5 * c_N[i] * (
					(deltas[i] + deltas[(i + 1) % length]) * (deltas[i] + deltas[(i + 1) % length])
					- 2.0 * deltas_pre[i] * deltas[i] - 2.0 * deltas_pre[(i + 1) % length] * deltas[(i + 1) % length]
					+ deltas_pre[i] * deltas_pre[i] + deltas_pre[(i + 1) % length] * deltas_pre[(i + 1) % length]); // q_plus
				A_plus_linear += 0.5 * c_N[i] * (2.0 * (
					deltas_pre[i] + deltas_pre[(i + 1) % length]) * (deltas[i] + deltas[(i + 1) % length])
					- deltas[i] * deltas[i] - deltas[(i + 1) % length] * deltas[(i + 1) % length]
					- (deltas_pre[i] + deltas_pre[(i + 1) % length]) * (deltas_pre[i] + deltas_pre[(i + 1) % length])); // q_minus
			}
		}
		A_plus_linear.addTerms(c_A, deltas, length);
		A_minus_linear.addTerms(c_A, deltas, length);
		GRBVar A_plus = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		GRBVar A_minus = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addQConstr(A_plus >= A_plus_linear);
		model.addQConstr(A_minus <= A_minus_linear);

		GRBVar* Dx = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, length);
		GRBVar* Dy = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, length);
		GRBVar* Dl = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, length);
		for (register int i = 0; i < length; ++i) {
			model.addConstr(Dx[i] >= x[i] + nx[i] * deltas[i] - x[(i + 1) % length] - nx[(i + 1) % length] * deltas[(i + 1) % length]);
			model.addConstr(Dx[i] >= -x[i] - nx[i] * deltas[i] + x[(i + 1) % length] + nx[(i + 1) % length] * deltas[(i + 1) % length]);
			model.addConstr(Dy[i] >= y[i] + ny[i] * deltas[i] - y[(i + 1) % length] - ny[(i + 1) % length] * deltas[(i + 1) % length]);
			model.addConstr(Dy[i] >= -y[i] - ny[i] * deltas[i] + y[(i + 1) % length] + ny[(i + 1) % length] * deltas[(i + 1) % length]);
			model.addQConstr(Dl[i] * Dl[i] >= Dx[i] * Dx[i] + Dy[i] * Dy[i]);
		}
		GRBLinExpr L_expr;
		L_expr.addTerms(one_coeffs, Dl, length);
		GRBVar L = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addConstr(L >= L_expr);
		GRBVar L2 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addQConstr(L2 >= L * L);

		GRBVar T = model.addVar(1.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
		model.addQConstr(L2 <= 2.0 * pi * T_pre * A_minus_pre * (4.0 * (T / T_pre + A_minus / A_minus_pre) - T * T / (T_pre * T_pre) - A_minus * A_minus / (A_minus_pre * A_minus_pre) - 4.0));


		GRBLinExpr obj = lambdaQ * T + lambdaS * A_plus / A0 + lambdaL * L / L0;
		//model.set(GRB_IntParam_NonConvex, 0);
		model.setObjective(obj, GRB_MINIMIZE);
		model.optimize();

		double Delta_delta = abs(deltas[0].get(GRB_DoubleAttr_X) - deltas_pre[0]);

		for (register int i = 0; i < length; ++i) {
			Delta_delta = max(Delta_delta, abs(deltas[i].get(GRB_DoubleAttr_X) - deltas_pre[i]));
			deltas_pre[i] = deltas[i].get(GRB_DoubleAttr_X);
		}
		// A_plus_pre = A_plus.get(GRB_DoubleAttr_X);
		A_minus_pre = A_minus.get(GRB_DoubleAttr_X);
		T_pre = T.get(GRB_DoubleAttr_X);

		Curve::OffsetNaive(x, y, deltas_pre, length, xtilde_pre, ytilde_pre, nx, ny);
		double L_now = Curve::TotalLength(xtilde_pre, ytilde_pre, length, true);
		double A_now = Curve::AreaCal(xtilde_pre, ytilde_pre, length);
		double T_now = L_now * L_now / (4 * pi * A_now);
		T_all[i_iter + 1] = lambdaQ * T_pre + lambdaS * A_plus_pre / A0 + lambdaL * L_pre / L0;

		cout << "After " << i_iter + 1 << " iterations, Q is " << T_now << ", A is " << A_now << ", L is " << L_now << ", and deltas varies " << Delta_delta << " now.\n";
		if ((i_iter > 3) && (Delta_delta < err)) {
			/*
			static int num = 0;
			++num;
			if (num == 10) {
				cout << "Here!\n";
			}*/
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
	delete[] T_all;
	return deltas_pre;
}

void NonEquidistant::addcons_ddot_deltas(GRBModel& model, double const* x, double const* y, GRBVar* deltas, int length, double dot_delta, double ddot_delta) {
	double* dl = Curve::DiffLength(x, y, length, true);
	for (register int i = 0; i < length; ++i) {
		model.addConstr(-dl[i] * dl[i] * deltas[(i + length - 1) % length]
			+ (dl[i] * dl[i] - dl[(i + length - 1) % length] * dl[(i + length - 1) % length]) * deltas[i]
			+ dl[(i + length - 1) % length] * dl[(i + length - 1) % length] * deltas[(i + 1) % length]
			<= dot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
		model.addConstr(-dl[i] * dl[i] * deltas[(i + length - 1) % length]
			+ (dl[i] * dl[i] - dl[(i + length - 1) % length] * dl[(i + length - 1) % length]) * deltas[i]
			+ dl[(i + length - 1) % length] * dl[(i + length - 1) % length] * deltas[(i + 1) % length]
			>= -dot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
		model.addConstr(dl[i] * deltas[(i + length - 1) % length]
			- (dl[(i + length - 1) % length] + dl[i]) * deltas[i]
			+ dl[(i + length - 1) % length] * deltas[(i + 1) % length]
			<= 0.5 * ddot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
		model.addConstr(dl[i] * deltas[(i + length - 1) % length]
			- (dl[(i + length - 1) % length] + dl[i]) * deltas[i]
			+ dl[(i + length - 1) % length] * deltas[(i + 1) % length]
			>= -0.5 * ddot_delta * dl[(i + length - 1) % length] * dl[i] * (dl[(i + length - 1) % length] + dl[i]));
	}
	delete[] dl;
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

// 多条非等距路径段的公有接口
paths NonEquidistant::NEpaths(const path& contour, const paths& holes, const NonEquidistantOptions& opts) {
	pathnode* root = root_offset(contour, holes, opts);
	//ContourParallel::clearvoid(root, holes, opts.delta, pi * opts.delta * opts.delta * 0.05, 0.02);
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
			paths ps2 = ContourParallel::OffsetClipper(ps1[i_path].x, ps1[i_path].y, -opts.delta * 1.001, ps1[i_path].length, opts.wash, min(opts.washdis, Length * 1.0 / opts.num_least)); // 必然只有一条path
			for (int j = 0; j < ps2.size(); ++j) {
				double lambda = 0.1;
				double* deltas = Optimize_QSL(p, opts);
				//double* deltas = IQA_optimize(ps2[j].x, ps2[j].y, ps2[j].length, opts.delta, opts.dot_delta, opts.ddot_delta, opts.alpha, lambda);
				// double* deltas = L2A_optimize(ps2[j].x, ps2[j].y, ps2[j].length, delta, dot_delta, ddot_delta, alpha);
				// double* deltas = L_optimize(ps2[j].x, ps2[j].y, ps2[j].length, delta, dot_delta, ddot_delta, alpha);
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