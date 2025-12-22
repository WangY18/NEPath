#include "nepath.h"

// Example function to demonstrate bindings
int add(int a, int b) {
    return a + b;
}

// Example function using Eigen
Eigen::VectorXd multiply_vector(const Eigen::VectorXd& vec, double scalar) {
    return vec * scalar;
}

NB_MODULE(_example, m) {
    m.doc() = "Example NEPath bindings module";

    m.def("add", &add, "a"_a, "b"_a = 1, "Add two numbers");

    m.def("multiply_vector", &multiply_vector,
          "vec"_a, "scalar"_a,
          "Multiply an Eigen vector by a scalar");
}
