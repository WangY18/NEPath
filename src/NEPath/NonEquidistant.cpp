#include <NEPath/NonEquidistant.h>
#include <NEPath/Basic.h>
#include <NEPath/ContourParallel.h>
#include <NEPath/Connector.h>
#include <cmath>
#include <iostream>

// The authors would like to sincerely thank Jelle Feringa for his significant contribution to the Ipopt optimization part of NEPath.

namespace nepath
{
    // ============================================================================
    // NonEquidistant Class Implementation
    // ============================================================================

    NonEquidistant::NonEquidistant(bool debug) : debug_(debug)
    {
    }

    NonEquidistant::~NonEquidistant()
    {
#if defined(IncludeGurobi) && (IncludeGurobi != 0)
        if (gurobi_)
        {
            delete gurobi_;
        }
#endif
    }

    double *NonEquidistant::Optimize_QSL(path p, const NonEquidistantOptions &opts)
    {
        switch (opts.optimizer)
        {
        case OptimizationAlgorithm::ipopt:
            return optimize_by_ipopt(p, opts);
        case OptimizationAlgorithm::gurobi:
            return optimize_by_gurobi(p, opts);
        case OptimizationAlgorithm::none:
            throw std::runtime_error("Error: No optimizer is selected in NonEquidistant::Optimize_QSL.");
        default:
            throw std::runtime_error("Error: Unknown optimizer is selected in NonEquidistant::Optimize_QSL.");
        }
        return nullptr;
    }

    pathnode *NonEquidistant::root_offset(const path &contour, const paths &holes, const NonEquidistantOptions &opts)
    {
        pathnode *root = new pathnode(contour);
        root->parent = NULL;
        std::stack<pathnode *> S;
        S.push(root);

        while (!S.empty())
        {
            pathnode *pn_parent = S.top();
            S.pop();
            paths ps_offset = do1offset(pn_parent->data, opts);
            for (int i_offset = 0; i_offset < ps_offset.size(); ++i_offset)
            {
                paths ps_clip = ContourParallel::cut_holes(ps_offset[i_offset], holes, true, 0.1);
                for (int i_clip = 0; i_clip < ps_clip.size(); ++i_clip)
                {
                    path p = opts.wash ? Curve::wash_dis(ps_clip[i_clip], opts.washdis, opts.num_least) : ps_clip[i_clip];
                    pathnode *pn_child = new pathnode(p);
                    pn_parent->children.push_back(pn_child);
                    pn_child->parent = pn_parent;
                    S.push(pn_child);
                }
            }
        }
        return root;
    }

    paths NonEquidistant::NEpaths(const path &contour, const paths &holes, const NonEquidistantOptions &opts)
    {
        pathnode *root = root_offset(contour, holes, opts);
        ContourParallel::clearvoid(root, holes, opts.delta, M_PI * opts.delta * opts.delta * 0.05, 0.02);
        std::vector<pathnode *> *dfs = pathnode::DFS_root(root);
        paths solution;
        for (int i = 0; i < dfs->size(); ++i)
        {
            solution.push_back(path());
            solution[i].length = (*dfs)[i]->data.length;
            solution[i].x = (*dfs)[i]->data.x;
            solution[i].y = (*dfs)[i]->data.y;
            (*dfs)[i]->data.clear_without_delete();
            for (int j = 0; j < (*dfs)[i]->children.size(); ++j)
            {
                (*dfs)[i]->children[j] = NULL;
            }
        }
        delete dfs;
        delete root;
        return solution;
    }

    paths NonEquidistant::do1offset(const path &p, const NonEquidistantOptions &opts)
    {
        paths ps;
        double Length = Curve::TotalLength(p.x, p.y, p.length);
        paths ps1 = ContourParallel::OffsetClipper(p.x, p.y, opts.delta * 1.001, p.length, opts.wash, std::min(opts.washdis, Length * 1.0 / opts.num_least));
        if (ps1.size())
        {
            for (int i_path = 0; i_path < ps1.size(); ++i_path)
            {
                Length = Curve::TotalLength(ps1[i_path].x, ps1[i_path].y, ps1[i_path].length);
                paths ps2 = ContourParallel::OffsetClipper(ps1[i_path].x, ps1[i_path].y, -opts.delta * 1.001, ps1[i_path].length, opts.wash, std::min(opts.washdis, Length * 1.0 / opts.num_least));
                for (int j = 0; j < ps2.size(); ++j)
                {
                    double *deltas = Optimize_QSL(ps2[j], opts);
                    if (deltas)
                    {
                        double max_deltas = deltas[0];
                        for (int i = 1; i < ps2[j].length; ++i)
                        {
                            max_deltas = std::max(max_deltas, deltas[i]);
                        }
                        for (int i = 0; i < ps2[j].length; ++i)
                        {
                            deltas[i] += opts.delta - max_deltas;
                        }
                        ps.push_back(path());
                        ps[ps.size() - 1].length = ps2[j].length;
                        Curve::OffsetNaive(ps2[j].x, ps2[j].y, deltas, ps2[j].length, ps[ps.size() - 1].x, ps[ps.size() - 1].y);
                        delete[] deltas;
                    }
                    else
                    {
                        paths ps3 = ContourParallel::OffsetClipper(ps2[j], paths(), opts.delta, opts.wash, opts.washdis, opts.num_least);
                        if (ps3.size() > 0)
                        {
                            ps.push_back(ps3[0]);
                        }
                    }
                }
            }
        }
        return ps;
    }

    path NonEquidistant::NEpaths_CFS(const path &contour, const paths &holes, const NonEquidistantOptions &opts)
    {
        pathnode *root = root_offset(contour, holes, opts);
        ContourParallel::clearvoid(root, holes, opts.delta, M_PI * opts.delta * opts.delta * 0.09, 0.02);
        return Connector::ConnectedFermatSpiral_MultMinimum(root, opts.delta);
    }

    path NonEquidistant::NEpaths_DFS(const path &contour, const paths &holes, const NonEquidistantOptions &opts)
    {
        pathnode *root = root_offset(contour, holes, opts);
        path p = Connector::ConnectedDFS(root);
        delete root;
        return opts.wash ? p : Curve::wash_dis(p, opts.washdis, opts.num_least);
    }

    double *NonEquidistant::optimize_by_ipopt(const path &p, const NonEquidistantOptions &opts)
    {
#if defined(IncludeIpopt) && (IncludeIpopt != 0)
        // Create IPOPT application
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

        // Set IPOPT options
        if (!debug_)
        {
            app->Options()->SetIntegerValue("print_level", 0);
        }
        else
        {
            app->Options()->SetIntegerValue("print_level", 5);
        }

        app->Options()->SetNumericValue("tol", opts.epsilon);
        app->Options()->SetIntegerValue("max_iter", opts.step_max * 100);
        app->Options()->SetStringValue("mu_strategy", "adaptive");
        app->Options()->SetStringValue("hessian_approximation", "limited-memory");

        // Initialize IPOPT
        Ipopt::ApplicationReturnStatus status = app->Initialize();
        if (status != Ipopt::Solve_Succeeded)
        {
            std::cout << "IPOPT initialization failed!" << std::endl;
            return nullptr;
        }

        // Create NLP problem
        Ipopt::SmartPtr<IQOP_NLP> nlp = new IQOP_NLP(p, opts, debug_);

        // Solve
        status = app->OptimizeTNLP(nlp);

        if (status == Ipopt::Solve_Succeeded || status == Ipopt::Solved_To_Acceptable_Level)
        {
            // Extract solution
            double *solution = new double[p.length];
            double *nlp_solution = nlp->get_solution();
            for (int i = 0; i < p.length; ++i)
            {
                solution[i] = nlp_solution[i];
            }
            return solution;
        }
        else
        {
            std::cout << "IPOPT optimization failed with status " << status << std::endl;
            return nullptr;
        }
#else
        std::cout << "IPOPT support is disabled!" << std::endl;
        return nullptr;
#endif
    }

    double *NonEquidistant::optimize_by_gurobi(const path &p, const NonEquidistantOptions &opts)
    {
#if defined(IncludeGurobi) && (IncludeGurobi != 0)
        if (!gurobi_)
        {
            gurobi_ = new GRBEnv(true);
            gurobi_->set(GRB_IntParam::GRB_IntParam_OutputFlag, debug_);
            gurobi_->start();
        }
        else
        {
            gurobi_->set(GRB_IntParam::GRB_IntParam_OutputFlag, debug_);
        }
        // constants
        double *deltas_pre = new double[p.length]();
        for (int i = 0; i < p.length; ++i)
        {
            deltas_pre[i] = 0.5 * (1.0 + opts.alpha) * opts.delta;
        }

        double A0 = Curve::AreaCal(p.x, p.y, p.length);
        double L0 = Curve::TotalLength(p.x, p.y, p.length, true);

        double *nx = NULL;
        double *ny = NULL;
        Curve::Ndir(p.x, p.y, p.length, nx, ny); // inward normal vector

        double *xtilde_pre = NULL;
        double *ytilde_pre = NULL;
        double *dl = NULL;

        // begin iteration
        Curve::OffsetNaive(p.x, p.y, deltas_pre, p.length, xtilde_pre, ytilde_pre, nx, ny);

        double *c_A = new double[p.length];
        double *c_N = new double[p.length];
        for (int i = 0; i < p.length; ++i)
        {
            c_A[i] = 0.5 * ((p.y[(i + 1) % p.length] - p.y[(i + p.length - 1) % p.length]) * nx[i] - (p.x[(i + 1) % p.length] - p.x[(i + p.length - 1) % p.length]) * ny[i]);
            c_N[i] = 0.5 * (ny[(i + 1) % p.length] * nx[i] - nx[(i + 1) % p.length] * ny[i]);
        }

        double *ub_deltas = new double[p.length];
        for (int i = 0; i < p.length; ++i)
        {
            ub_deltas[i] = opts.delta;
        }
        double *ub_inf = new double[p.length];
        for (int i = 0; i < p.length; ++i)
        {
            ub_inf[i] = GRB_INFINITY;
        }
        double *one_coeffs = new double[p.length];
        double *alpha_coeffs = new double[p.length];
        for (int i = 0; i < p.length; ++i)
        {
            one_coeffs[i] = 1.0;
            alpha_coeffs[i] = opts.alpha * opts.delta;
        }

        // variables
        for (int i_iter = 0; i_iter < opts.step_max; ++i_iter)
        {
            GRBModel model = GRBModel(*gurobi_);
            GRBVar *deltas = model.addVars(alpha_coeffs, ub_deltas, NULL, NULL, NULL, NULL, p.length);
            Curve::DiffLength(dl, p.x, p.y, p.length, true);
            for (int i = 0; i < p.length; ++i)
            {
                model.addConstr(-dl[i] * dl[i] * deltas[(i + p.length - 1) % p.length] + (dl[i] * dl[i] - dl[(i + p.length - 1) % p.length] * dl[(i + p.length - 1) % p.length]) * deltas[i] + dl[(i + p.length - 1) % p.length] * dl[(i + p.length - 1) % p.length] * deltas[(i + 1) % p.length] <= opts.dot_delta * dl[(i + p.length - 1) % p.length] * dl[i] * (dl[(i + p.length - 1) % p.length] + dl[i]));
                model.addConstr(-dl[i] * dl[i] * deltas[(i + p.length - 1) % p.length] + (dl[i] * dl[i] - dl[(i + p.length - 1) % p.length] * dl[(i + p.length - 1) % p.length]) * deltas[i] + dl[(i + p.length - 1) % p.length] * dl[(i + p.length - 1) % p.length] * deltas[(i + 1) % p.length] >= -opts.dot_delta * dl[(i + p.length - 1) % p.length] * dl[i] * (dl[(i + p.length - 1) % p.length] + dl[i]));
                model.addConstr(dl[i] * deltas[(i + p.length - 1) % p.length] - (dl[(i + p.length - 1) % p.length] + dl[i]) * deltas[i] + dl[(i + p.length - 1) % p.length] * deltas[(i + 1) % p.length] <= 0.5 * opts.ddot_delta * dl[(i + p.length - 1) % p.length] * dl[i] * (dl[(i + p.length - 1) % p.length] + dl[i]));
                model.addConstr(dl[i] * deltas[(i + p.length - 1) % p.length] - (dl[(i + p.length - 1) % p.length] + dl[i]) * deltas[i] + dl[(i + p.length - 1) % p.length] * deltas[(i + 1) % p.length] >= -0.5 * opts.ddot_delta * dl[(i + p.length - 1) % p.length] * dl[i] * (dl[(i + p.length - 1) % p.length] + dl[i]));
            }

            GRBQuadExpr A_plus_linear = A0;
            GRBQuadExpr A_minus_linear = A0;
            for (int i = 0; i < p.length; ++i)
            {
                if (c_N[i] >= 0)
                {
                    if (opts.optimize_Q)
                    {
                        A_minus_linear += 0.5 * c_N[i] * (2.0 * (deltas_pre[i] + deltas_pre[(i + 1) % p.length]) * (deltas[i] + deltas[(i + 1) % p.length]) - deltas[i] * deltas[i] - deltas[(i + 1) % p.length] * deltas[(i + 1) % p.length] - (deltas_pre[i] + deltas_pre[(i + 1) % p.length]) * (deltas_pre[i] + deltas_pre[(i + 1) % p.length])); // q_minus
                    }
                    if (opts.optimize_S)
                    {
                        A_plus_linear += 0.5 * c_N[i] * ((deltas[i] + deltas[(i + 1) % p.length]) * (deltas[i] + deltas[(i + 1) % p.length]) - 2.0 * deltas_pre[i] * deltas[i] - 2.0 * deltas_pre[(i + 1) % p.length] * deltas[(i + 1) % p.length] + deltas_pre[i] * deltas_pre[i] + deltas_pre[(i + 1) % p.length] * deltas_pre[(i + 1) % p.length]); // q_plus
                    }
                }
                else
                {
                    if (opts.optimize_Q)
                    {
                        A_minus_linear += 0.5 * c_N[i] * ((deltas[i] + deltas[(i + 1) % p.length]) * (deltas[i] + deltas[(i + 1) % p.length]) - 2.0 * deltas_pre[i] * deltas[i] - 2.0 * deltas_pre[(i + 1) % p.length] * deltas[(i + 1) % p.length] + deltas_pre[i] * deltas_pre[i] + deltas_pre[(i + 1) % p.length] * deltas_pre[(i + 1) % p.length]); // q_plus
                    }
                    if (opts.optimize_S)
                    {
                        A_plus_linear += 0.5 * c_N[i] * (2.0 * (deltas_pre[i] + deltas_pre[(i + 1) % p.length]) * (deltas[i] + deltas[(i + 1) % p.length]) - deltas[i] * deltas[i] - deltas[(i + 1) % p.length] * deltas[(i + 1) % p.length] - (deltas_pre[i] + deltas_pre[(i + 1) % p.length]) * (deltas_pre[i] + deltas_pre[(i + 1) % p.length])); // q_minus
                    }
                }
            }
            GRBVar A_plus = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            GRBVar A_minus = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            if (opts.optimize_S)
            {
                A_plus_linear.addTerms(c_A, deltas, p.length);
                model.addQConstr(A_plus >= A_plus_linear);
            }
            if (opts.optimize_Q)
            {
                A_minus_linear.addTerms(c_A, deltas, p.length);
                model.addQConstr(A_minus <= A_minus_linear);
            }

            GRBVar *Dx = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, p.length);
            GRBVar *Dy = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, p.length);
            GRBVar *Dl = model.addVars(NULL, ub_inf, NULL, NULL, NULL, NULL, p.length);
            for (int i = 0; i < p.length; ++i)
            {
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
            if (opts.optimize_Q)
            {
                model.addQConstr(L * L + pi * A0 * (Q - A_minus / A0) * (Q - A_minus / A0) <= pi * A0 * (Q + A_minus / A0) * (Q + A_minus / A0));
            }

            GRBLinExpr obj = 0;
            if (opts.optimize_L)
            {
                obj += opts.lambda_L * L / L0;
            }
            if (opts.optimize_Q)
            {
                obj += opts.lambda_Q * Q;
            }
            if (opts.optimize_S)
            {
                obj += opts.lambda_S * A_plus / A0;
            }

            model.set(GRB_IntParam_NonConvex, 0);
            model.setObjective(obj, GRB_MINIMIZE);
            model.optimize();

            double Delta_delta = abs(deltas[0].get(GRB_DoubleAttr_X) - deltas_pre[0]);

            for (int i = 0; i < p.length; ++i)
            {
                Delta_delta = std::max(Delta_delta, fabs(deltas[i].get(GRB_DoubleAttr_X) - deltas_pre[i]));
                deltas_pre[i] = deltas[i].get(GRB_DoubleAttr_X);
            }

            Curve::OffsetNaive(p.x, p.y, deltas_pre, p.length, xtilde_pre, ytilde_pre, nx, ny);
            double L_now = Curve::TotalLength(xtilde_pre, ytilde_pre, p.length, true);
            double A_now = Curve::AreaCal(xtilde_pre, ytilde_pre, p.length);
            double Q_now = L_now * L_now / (4 * pi * A_now);

            if (debug_)
            {
                std::cout << "After " << i_iter + 1 << " iterations, Q is " << Q_now << ", A is " << A_now << ", L is " << L_now << ", and deltas varies " << Delta_delta << " now.\n";
            }
            if ((!opts.optimize_Q && !opts.optimize_S) || ((i_iter > 1) && (Delta_delta < opts.epsilon)) || (i_iter > opts.step_max))
            {
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
#else
        std::cout << "Gurobi support is disabled!" << std::endl;
        return nullptr;
#endif
    }
}

// ============================================================================
// IQOP_NLP Implementation - IPOPT Nonlinear Problem Definition
// ============================================================================
#if defined(IncludeIpopt) && (IncludeIpopt != 0)
namespace nepath
{
    IQOP_NLP::IQOP_NLP(const path &p, const NonEquidistantOptions &opts, bool output_debug /*=false*/)
        : p_(p), opts_(opts), n_vars_(p.length), solution_deltas(nullptr), debug_(output_debug)
    {
        // Initialize solution storage
        solution_deltas = new double[n_vars_];
        deltas_pre_ = new double[n_vars_];

        // Initialize with mid-range values
        for (int i = 0; i < n_vars_; ++i)
        {
            deltas_pre_[i] = 0.5 * (1.0 + opts_.alpha) * opts_.delta;
            solution_deltas[i] = deltas_pre_[i];
        }

        // Pre-compute geometric constants
        A0_ = Curve::AreaCal(p_.x, p_.y, p_.length);
        L0_ = Curve::TotalLength(p_.x, p_.y, p_.length, true);

        // Compute normal directions
        nx_ = nullptr;
        ny_ = nullptr;
        Curve::Ndir(p_.x, p_.y, p_.length, nx_, ny_);

        // Compute edge lengths
        dl_ = Curve::DiffLength(p_.x, p_.y, p_.length, true);

        // Compute area and normal coefficients
        c_A_ = new double[p_.length];
        c_N_ = new double[p_.length];
        for (int i = 0; i < p_.length; ++i)
        {
            c_A_[i] = 0.5 * ((p_.y[(i + 1) % p_.length] - p_.y[(i + p_.length - 1) % p_.length]) * nx_[i] - (p_.x[(i + 1) % p_.length] - p_.x[(i + p_.length - 1) % p_.length]) * ny_[i]);
            c_N_[i] = 0.5 * (ny_[(i + 1) % p_.length] * nx_[i] - nx_[(i + 1) % p_.length] * ny_[i]);
        }

        // Count constraints:
        // - 2*n_vars_ for dot_delta constraints (2 per edge: upper and lower)
        // - 2*n_vars_ for ddot_delta constraints (2 per edge: upper and lower)
        // Total: 4*n_vars_ linear constraints
        n_cons_ = 4 * n_vars_;
    }

    IQOP_NLP::~IQOP_NLP()
    {
        delete[] solution_deltas;
        delete[] deltas_pre_;
        delete[] nx_;
        delete[] ny_;
        delete[] dl_;
        delete[] c_A_;
        delete[] c_N_;
    }

    bool IQOP_NLP::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                                Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        // Ipopt::Number of variables (deltas)
        n = n_vars_;

        // Ipopt::Number of constraints
        m = n_cons_;

        // Ipopt::Number of nonzeros in Jacobian
        // Each constraint involves 3 consecutive deltas
        nnz_jac_g = 3 * n_cons_;

        // Ipopt::Number of nonzeros in Hessian
        // We use limited-memory approximation, so no Hessian structure needed
        nnz_h_lag = 0;

        // Use C-style indexing (0-based)
        index_style = TNLP::C_STYLE;

        return true;
    }

    bool IQOP_NLP::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                                   Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        // Variable bounds: alpha*delta <= delta_i <= delta
        for (Ipopt::Index i = 0; i < n; ++i)
        {
            x_l[i] = opts_.alpha * opts_.delta;
            x_u[i] = opts_.delta;
        }

        // Constraint bounds: all are inequality constraints of form g(x) <= 0
        for (Ipopt::Index i = 0; i < m; ++i)
        {
            g_l[i] = -1e20; // No lower bound
            g_u[i] = 0.0;   // Upper bound at 0
        }

        return true;
    }

    bool IQOP_NLP::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                                      bool /*init_z*/, Ipopt::Number * /*z_L*/, Ipopt::Number * /*z_U*/,
                                      Ipopt::Index /*m*/, bool /*init_lambda*/, Ipopt::Number * /*lambda*/)
    {
        // Initialize primal variables
        if (init_x)
        {
            for (Ipopt::Index i = 0; i < n; ++i)
            {
                x[i] = deltas_pre_[i];
            }
        }

        // We don't initialize dual variables
        return true;
    }

    bool IQOP_NLP::eval_f(Ipopt::Index /*n*/, const Ipopt::Number *x, bool /*new_x*/, Ipopt::Number &obj_value)
    {
        // Compute objective function
        obj_value = 0.0;

        // Compute offset path properties
        double *xtilde = nullptr;
        double *ytilde = nullptr;
        Curve::OffsetNaive(p_.x, p_.y, x, p_.length, xtilde, ytilde, nx_, ny_);

        double L_now = Curve::TotalLength(xtilde, ytilde, p_.length, true);
        double A_now = Curve::AreaCal(xtilde, ytilde, p_.length);
        double Q_now = (A_now > 1e-10) ? (L_now * L_now / (4.0 * M_PI * A_now)) : 1e10;

        // Objective: weighted sum of Q, S, L
        if (opts_.optimize_Q)
        {
            obj_value += opts_.lambda_Q * Q_now;
        }
        if (opts_.optimize_S)
        {
            obj_value += opts_.lambda_S * A_now / A0_;
        }
        if (opts_.optimize_L)
        {
            obj_value += opts_.lambda_L * L_now / L0_;
        }

        delete[] xtilde;
        delete[] ytilde;

        return true;
    }

    bool IQOP_NLP::eval_grad_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number *grad_f)
    {
        // Compute gradient using finite differences
        const double epsilon = 1e-7;

        double f0;
        eval_f(n, x, new_x, f0);

        for (Ipopt::Index i = 0; i < n; ++i)
        {
            double *x_plus = new double[n];
            for (Ipopt::Index j = 0; j < n; ++j)
            {
                x_plus[j] = x[j];
            }
            x_plus[i] += epsilon;

            double f_plus;
            eval_f(n, x_plus, true, f_plus);

            grad_f[i] = (f_plus - f0) / epsilon;

            delete[] x_plus;
        }

        return true;
    }

    bool IQOP_NLP::eval_g(Ipopt::Index /*n*/, const Ipopt::Number *x, bool /*new_x*/,
                          Ipopt::Index /*m*/, Ipopt::Number *g)
    {
        // Evaluate constraints
        int con_idx = 0;

        // dot_delta constraints (4*n_vars_)
        for (int i = 0; i < n_vars_; ++i)
        {
            int i_prev = (i + n_vars_ - 1) % n_vars_;
            int i_next = (i + 1) % n_vars_;

            double dl_i = dl_[i];
            double dl_prev = dl_[i_prev];

            double lhs = -dl_i * dl_i * x[i_prev] + (dl_i * dl_i - dl_prev * dl_prev) * x[i] + dl_prev * dl_prev * x[i_next];

            double rhs_upper = opts_.dot_delta * dl_prev * dl_i * (dl_prev + dl_i);
            double rhs_lower = -opts_.dot_delta * dl_prev * dl_i * (dl_prev + dl_i);

            // Upper constraint: lhs <= rhs_upper  =>  lhs - rhs_upper <= 0
            g[con_idx++] = lhs - rhs_upper;

            // Lower constraint: lhs >= rhs_lower  =>  rhs_lower - lhs <= 0
            g[con_idx++] = rhs_lower - lhs;
        }

        // ddot_delta constraints (4*n_vars_)
        for (int i = 0; i < n_vars_; ++i)
        {
            int i_prev = (i + n_vars_ - 1) % n_vars_;
            int i_next = (i + 1) % n_vars_;

            double dl_i = dl_[i];
            double dl_prev = dl_[i_prev];

            double lhs = dl_i * x[i_prev] - (dl_prev + dl_i) * x[i] + dl_prev * x[i_next];

            double rhs_upper = 0.5 * opts_.ddot_delta * dl_prev * dl_i * (dl_prev + dl_i);
            double rhs_lower = -0.5 * opts_.ddot_delta * dl_prev * dl_i * (dl_prev + dl_i);

            // Upper constraint
            g[con_idx++] = lhs - rhs_upper;

            // Lower constraint
            g[con_idx++] = rhs_lower - lhs;
        }

        return true;
    }

    bool IQOP_NLP::eval_jac_g(Ipopt::Index /*n*/, const Ipopt::Number * /*x*/, bool /*new_x*/,
                              Ipopt::Index /*m*/, Ipopt::Index /*nele_jac*/, Ipopt::Index *iRow,
                              Ipopt::Index *jCol, Ipopt::Number *values)
    {
        if (values == nullptr)
        {
            // Return structure of Jacobian
            int idx = 0;

            // dot_delta constraints
            for (int i = 0; i < n_vars_; ++i)
            {
                int i_prev = (i + n_vars_ - 1) % n_vars_;
                int i_next = (i + 1) % n_vars_;

                // Upper constraint
                iRow[idx] = 2 * i;
                jCol[idx] = i_prev;
                idx++;

                iRow[idx] = 2 * i;
                jCol[idx] = i;
                idx++;

                iRow[idx] = 2 * i;
                jCol[idx] = i_next;
                idx++;

                // Lower constraint
                iRow[idx] = 2 * i + 1;
                jCol[idx] = i_prev;
                idx++;

                iRow[idx] = 2 * i + 1;
                jCol[idx] = i;
                idx++;

                iRow[idx] = 2 * i + 1;
                jCol[idx] = i_next;
                idx++;
            }

            // ddot_delta constraints
            for (int i = 0; i < n_vars_; ++i)
            {
                int i_prev = (i + n_vars_ - 1) % n_vars_;
                int i_next = (i + 1) % n_vars_;
                int base_con = 2 * n_vars_;

                // Upper constraint
                iRow[idx] = base_con + 2 * i;
                jCol[idx] = i_prev;
                idx++;

                iRow[idx] = base_con + 2 * i;
                jCol[idx] = i;
                idx++;

                iRow[idx] = base_con + 2 * i;
                jCol[idx] = i_next;
                idx++;

                // Lower constraint
                iRow[idx] = base_con + 2 * i + 1;
                jCol[idx] = i_prev;
                idx++;

                iRow[idx] = base_con + 2 * i + 1;
                jCol[idx] = i;
                idx++;

                iRow[idx] = base_con + 2 * i + 1;
                jCol[idx] = i_next;
                idx++;
            }
        }
        else
        {
            // Return values of Jacobian
            int idx = 0;

            // dot_delta constraints
            for (int i = 0; i < n_vars_; ++i)
            {
                int i_prev = (i + n_vars_ - 1) % n_vars_;

                double dl_i = dl_[i];
                double dl_prev = dl_[i_prev];

                // Upper constraint coefficients
                values[idx++] = -dl_i * dl_i;                    // x[i_prev]
                values[idx++] = dl_i * dl_i - dl_prev * dl_prev; // x[i]
                values[idx++] = dl_prev * dl_prev;               // x[i_next]

                // Lower constraint coefficients (negated)
                values[idx++] = dl_i * dl_i;                        // x[i_prev]
                values[idx++] = -(dl_i * dl_i - dl_prev * dl_prev); // x[i]
                values[idx++] = -dl_prev * dl_prev;                 // x[i_next]
            }

            // ddot_delta constraints
            for (int i = 0; i < n_vars_; ++i)
            {
                int i_prev = (i + n_vars_ - 1) % n_vars_;

                double dl_i = dl_[i];
                double dl_prev = dl_[i_prev];

                // Upper constraint coefficients
                values[idx++] = dl_i;              // x[i_prev]
                values[idx++] = -(dl_prev + dl_i); // x[i]
                values[idx++] = dl_prev;           // x[i_next]

                // Lower constraint coefficients (negated)
                values[idx++] = -dl_i;            // x[i_prev]
                values[idx++] = (dl_prev + dl_i); // x[i]
                values[idx++] = -dl_prev;         // x[i_next]
            }
        }

        return true;
    }

    bool IQOP_NLP::eval_h(Ipopt::Index /*n*/, const Ipopt::Number * /*x*/, bool /*new_x*/,
                          Ipopt::Number /*obj_factor*/, Ipopt::Index /*m*/, const Ipopt::Number * /*lambda*/,
                          bool /*new_lambda*/, Ipopt::Index /*nele_hess*/, Ipopt::Index * /*iRow*/,
                          Ipopt::Index * /*jCol*/, Ipopt::Number *values)
    {
        if (values == nullptr)
        {
            // Return structure of Hessian (use quasi-Newton approximation)
            // Return empty structure to use IPOPT's approximation
            return false; // Let IPOPT use quasi-Newton
        }

        return false; // Let IPOPT use quasi-Newton
    }

    void IQOP_NLP::finalize_solution(Ipopt::SolverReturn /*status*/, Ipopt::Index n,
                                     const Ipopt::Number *x, const Ipopt::Number * /*z_L*/,
                                     const Ipopt::Number * /*z_U*/, Ipopt::Index /*m*/,
                                     const Ipopt::Number * /*g*/, const Ipopt::Number * /*lambda*/,
                                     Ipopt::Number /*obj_value*/,
                                     const Ipopt::IpoptData * /*ip_data*/,
                                     Ipopt::IpoptCalculatedQuantities * /*ip_cq*/)
    {
        // Copy solution
        for (Ipopt::Index i = 0; i < n; ++i)
        {
            solution_deltas[i] = x[i];
        }

        if (debug_)
        {
            // Compute final metrics
            double *xtilde = nullptr;
            double *ytilde = nullptr;
            Curve::OffsetNaive(p_.x, p_.y, x, p_.length, xtilde, ytilde, nx_, ny_);
            double L_now = Curve::TotalLength(xtilde, ytilde, p_.length, true);
            double A_now = Curve::AreaCal(xtilde, ytilde, p_.length);
            double Q_now = (A_now > 1e-10) ? (L_now * L_now / (4.0 * M_PI * A_now)) : 1e10;
            std::cout << "IPOPT Solution: Q = " << Q_now
                      << ", A = " << A_now
                      << ", L = " << L_now << std::endl;
            delete[] xtilde;
            delete[] ytilde;
        }
    }
}
#endif // IncludeIpopt