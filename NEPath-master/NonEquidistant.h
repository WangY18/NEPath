#pragma once
#include "setup_NEPath.h"
// NonEquidistant is a class to plan non-equidistant toolpaths.

#if defined(IncludeIpopt) && (IncludeIpopt != 0)
#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"
#include "Curve.h"
#include "PlanningOptions.h"

// IPOPT NLP problem definition for IQOP optimization
class IQOP_NLP : public Ipopt::TNLP {
public:
    IQOP_NLP(const path& p, const NonEquidistantOptions& opts);
    virtual ~IQOP_NLP();

    // IPOPT required methods
    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m,
                            Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag,
                            IndexStyleEnum& index_style);

    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                               Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                   bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                   Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda);

    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                       Ipopt::Number& obj_value);

    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Number* grad_f);

    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                       Ipopt::Index m, Ipopt::Number* g);

    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                           Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                           Ipopt::Index* jCol, Ipopt::Number* values);

    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                       Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                       bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                       Ipopt::Index* jCol, Ipopt::Number* values);

    virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                                  const Ipopt::Number* x, const Ipopt::Number* z_L,
                                  const Ipopt::Number* z_U, Ipopt::Index m,
                                  const Ipopt::Number* g, const Ipopt::Number* lambda,
                                  Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data,
                                  Ipopt::IpoptCalculatedQuantities* ip_cq);

    double* get_solution() { return solution_deltas; }

private:
    path p_;  // Input path
    NonEquidistantOptions opts_;  // Options
    int n_vars_;  // Number of variables (deltas)
    int n_cons_;  // Number of constraints

    // Pre-computed constants
    double* nx_;  // Normal directions x
    double* ny_;  // Normal directions y
    double* dl_;  // Edge lengths
    double* c_A_; // Area coefficients
    double* c_N_; // Normal coefficients
    double A0_;   // Initial area
    double L0_;   // Initial length
    double* deltas_pre_;  // Previous iteration deltas

    double* solution_deltas;  // Final solution
};

class NonEquidistant {
public:
    NonEquidistant(bool debug = false);
    paths NEpaths(const path& contour, const paths& holes, const NonEquidistantOptions& opts);
    path NEpaths_CFS(const path& contour, const paths& holes, const NonEquidistantOptions& opts);
    path NEpaths_DFS(const path& contour, const paths& holes, const NonEquidistantOptions& opts);
private:
    double* Optimize_QSL(path p, const NonEquidistantOptions& opts);
    paths do1offset(const path& contour, const NonEquidistantOptions& opts);
    pathnode* root_offset(const path& contour, const paths& holes, const NonEquidistantOptions& opts);
private:
    bool debug_;
};
#endif
