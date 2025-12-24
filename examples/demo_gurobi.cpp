#include <NEPath/NEPathPlanner.h>
#include <NEPath/FileAgent.h>
#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;
using namespace std;
using namespace nepath;

int main()
{
    try
    {
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "gurobi_cpp_test.log");
        env.start();

        GRBModel model = GRBModel(env);
        model.set(GRB_StringAttr_ModelName, "lp_test");

        GRBVar x = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x");
        GRBVar y = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y");

        GRBLinExpr obj = 3 * x + 2 * y;
        model.setObjective(obj, GRB_MAXIMIZE);

        model.addConstr(x + y <= 4, "c1");
        model.addConstr(2 * x + y <= 5, "c2");

        model.optimize();

        int status = model.get(GRB_IntAttr_Status);
        if (status == GRB_OPTIMAL)
        {
            std::cout << "Optimal solution found.\n";
            std::cout << "x = " << x.get(GRB_DoubleAttr_X) << std::endl;
            std::cout << "y = " << y.get(GRB_DoubleAttr_X) << std::endl;
            std::cout << "Obj = " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
        }
        else
        {
            std::cout << "Optimization was not successful. Status = "
                      << status << std::endl;
        }
    }
    catch (GRBException &e)
    {
        std::cerr << "Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown error occurred." << std::endl;
    }

    return 0;
}