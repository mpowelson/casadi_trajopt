#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <casadi/casadi.hpp>
#include <casadi/core/shared_object.hpp>

using namespace casadi;
using namespace std;

int main()
{
  // Note that casadi is column major

  /** Note that there are two Symbolic representations MX and SX. It seems that SX represents each item in the matrix as
   * its own "variable". MX represents the entire matrix as one variable. The two can interact through MX Functions.
   * "The SX expressions are thus intended to be used for low level operations (for example the DAE right hand side in
   * Section 4.4), whereas the MX expressions act as a glue" */

  {
    std::cout << "Exploring SX... \n";
    SX x = SX::sym("x");
    SX y = SX::sym("y", 5);
    SX Z = SX::sym("Z", 4, 2);
    SX f = pow(x, 2) + 10;
    f = sqrt(f);
    std::cout << "f: " << f << std::endl;
    std::cout << "Z: " << Z << std::endl;
  }

  {
    std::cout << "\n\nExploring MX and Opti... \n";
    Opti opti = casadi::Opti();
    // Set up variables. These are each matrices. I just have them set to size 1
    MX x = opti.variable();
    MX y = opti.variable();
    MX z = opti.variable();

    // Parameters must be set/fixed at solve time
    MX p = opti.parameter();
    opti.set_value(p, 3);

    opti.minimize(pow((y - pow(x, 2)) + z - p, 2));
    opti.subject_to(pow(pow(x, 2) + y + z, 2) == 1);
    opti.subject_to((x + y) >= 1);
    opti.subject_to(z == 0);

    opti.solver("ipopt");
    OptiSol sol = opti.solve();

    std::cout << "objective func: " << opti.f() << std::endl;
    std::cout << "constraints: " << opti.g() << std::endl;

    std::cout << "Solution: ";
    sol.disp(std::cout);
    std::cout << "\n";
  }

  {
    std::cout << "\n\nExploring Functions... \n";
    casadi::Function function1;
    {
      SX x = SX::sym("x");
      SX f = pow(x, 2) + 10;
      std::vector<SX> func1_inputs = { x };
      std::vector<SX> func1_outputs = { f };
      function1 = casadi::Function("Function1", func1_inputs, func1_outputs);
    }
    casadi::Function function2;
    {
      // Set up variables. These are each matrices. I just have them set to size 1
      MX x = MX::sym("x", 2);
      MX y = MX::sym("y");
      MX f = x * sin(y);
      std::vector<MX> func2_inputs = { x, y };
      std::vector<MX> func2_outputs = { f };
      function2 = casadi::Function("Function2", func2_inputs, func2_outputs);
    }
    SX c = SX::sym("c");
    std::vector<SX> func1_inputs = { c };
    // This is another way to define a nonlinear solver. Opti is new
    std::vector<SX> output1 = function1(func1_inputs);

    MX a = MX::sym("a", 2);
    MX b = MX::sym("b");
    std::vector<MX> func2_inputs = { a, b };

    std::vector<MX> output2 = function2(func2_inputs);

    std::cout << "\nStill not sure how this really helps you. Somehow you can use it to embed SX functions in a MX "
                 "graph.... \n";
  }

  {
    std::cout << "\n\nNow we start the real TrajOpt example \n";

    // Define the inputs.
    SX joint_vals = SX::sym("joint_vals", 10, 7);
    std::cout << "jv.size1:" << joint_vals.size1() << " jv.size2: " << joint_vals.size2() << "\n";
    SX x;
    for (int ind2 = 0; ind2 < joint_vals.size2(); ind2++)
      for (int ind1 = 0; ind1 < joint_vals.size1(); ind1++)
      {
        {
          x = vertcat(x, joint_vals(ind1, ind2));
        }
      }

    std::cout << "x.size1:" << x.size1() << " x.size2: " << x.size2() << "\n";

    // Define the objective function (In this case just joint velocity)
    // Slice is similar to Python indexing, but I'm missing a row and a column here (TODO)
    SX slice0 = joint_vals(Slice(0, -2), Slice(0, -1));
    SX slice1 = joint_vals(Slice(1, -1), Slice(0, -1));
    // These are the joint velocities
    SX diff0 = slice1 - slice0;
    // Squre them for error from 0. This is element wise
    SX squared_vel = diff0 * diff0;

    // Define the constraints.
    // First we'll constrain the start and end points
    SX endpoint1 = joint_vals(0, Slice(0, -1));
    SX endpoint2 = joint_vals(joint_vals.size1() - 1, Slice(0, -1));
    // Next we'll constrain the velocities
    SX velocities = diff0;
    std::cout << "v.size1:" << velocities.size1() << " v.size2: " << velocities.size2() << "\n";

    // Form the constraint object
    //    endpoint1.resize(endpoint1.size2(),1);
    //    endpoint2.resize(endpoint2.size2(),1);
    //    velocities.resize(velocities.size1()*velocities.size2(),1);
    //    SX g = vertcat(endpoint1, endpoint2, velocities);

    // This is terrible. But I don't know what I'm supposed to do as an alternative. Above didn't seem to work
    SX g;
    for (int ind2 = 0; ind2 < velocities.size2(); ind2++)
      for (int ind1 = 0; ind1 < velocities.size1(); ind1++)
      {
        {
          g = vertcat(g, velocities(ind1, ind2));
        }
      }
//    // Start point
//    for (int ind2 = 0; ind2 < joint_vals.size2(); ind2++)
//    {
//      g = vertcat(g, joint_vals(0,ind2));
//    }
//    // Stop point
//    for (int ind2 = 0; ind2 < joint_vals.size2(); ind2++)
//    {
//      g = vertcat(g, joint_vals(joint_vals.size1()-1,ind2));
//    }
    std::cout << "g.size1:" << g.size1() << " g.size2: " << g.size2() << "\n";



    // Add the limits for the constraints. First 2 equality. Last one inequality
    vector<double> lbg(48, -0.1);
    vector<double> ubg(48, 0.1);
    // Initial guess and bounds for the optimization variables
    vector<double> x0(x.size1(), 0.0);
    vector<double> lbx(x.size1(), -inf);
    vector<double> ubx(x.size1(), inf);

    // A dummy parameter. Not sure if this is optional
    SX p = SX::sym("p", 2);
    vector<double> p0 = { 5.00, 1.00 };

    // Form NLP
    SXDict nlp = { { "x", x }, { "p", p }, { "f", squared_vel }, { "g", g } };

    std::cout << "Squared velocity: " << squared_vel << std::endl;
    std::cout << "\nConstraints: " << g << std::endl;

    // Create NLP solver and buffers
    Function solver = nlpsol("solver", "ipopt", nlp);
    std::map<std::string, DM> arg, res;

    // Solve the NLP
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    arg["x0"] = x0;
    arg["p"] = p0;
    res = solver(arg);

    // Print the solution
    cout << "-----" << endl;
    cout << "Optimal solution for p = " << arg.at("p") << ":" << endl;
    cout << setw(30) << "Objective: " << res.at("f") << endl;
    cout << setw(30) << "Primal solution: " << res.at("x") << endl;
    cout << setw(30) << "Dual solution (x): " << res.at("lam_x") << endl;
    cout << setw(30) << "Dual solution (g): " << res.at("lam_g") << endl;
  }

  // This is another way to define a nonlinear solver. Opti is new
  /** Test problem (Ganesh & Biegler, A reduced Hessian strategy for sensitivity analysis of optimal flowsheets, AIChE
   * 33, 1987, pp. 282-296)
   *
   *    min     x1^2 + x2^2 + x3^2
   *    s.t.    6*x1 + 3&x2 + 2*x3 - pi = 0
   *            p2*x1 + x2 - x3 - 1 = 0
   *            x1, x2, x3 >= 0
   *
   */
  //  {
  //    // Optimization variables
  //    SX x = SX::sym("x", 3);

  //    // Parameters
  //    SX p = SX::sym("p", 2);

  //    // Objective
  //    SX f = x(0) * x(0) + x(1) * x(1) + x(2) * x(2);

  //    // Constraints
  //    SX g = vertcat(6 * x(0) + 3 * x(1) + 2 * x(2) - p(0), p(1) * x(0) + x(1) - x(2) - 1);
  //    std::cout << g;

  //    // Initial guess and bounds for the optimization variables
  //    vector<double> x0 = { 0.15, 0.15, 0.00 };
  //    vector<double> lbx = { 0.00, 0.00, 0.00 };
  //    vector<double> ubx = { inf, inf, inf };

  //    // Nonlinear bounds
  //    vector<double> lbg = { 0.00, 0.00 };
  //    vector<double> ubg = { 0.00, 0.00 };

  //    // Original parameter values
  //    vector<double> p0 = { 5.00, 1.00 };

  //    // NLP
  //    SXDict nlp = { { "x", x }, { "p", p }, { "f", f }, { "g", g } };

  //    // Create NLP solver and buffers
  //    Function solver = nlpsol("solver", "ipopt", nlp);
  //    std::map<std::string, DM> arg, res;

  //    // Solve the NLP
  //    arg["lbx"] = lbx;
  //    arg["ubx"] = ubx;
  //    arg["lbg"] = lbg;
  //    arg["ubg"] = ubg;
  //    arg["x0"] = x0;
  //    arg["p"] = p0;
  //    res = solver(arg);

  //    // Print the solution
  //    cout << "-----" << endl;
  //    cout << "Optimal solution for p = " << arg.at("p") << ":" << endl;
  //    cout << setw(30) << "Objective: " << res.at("f") << endl;
  //    cout << setw(30) << "Primal solution: " << res.at("x") << endl;
  //    cout << setw(30) << "Dual solution (x): " << res.at("lam_x") << endl;
  //    cout << setw(30) << "Dual solution (g): " << res.at("lam_g") << endl;

  //    // Change the parameter and resolve
  //    arg["p"] = 4.5;
  //    res = solver(arg);

  //    // Print the new solution
  //    cout << "-----" << endl;
  //    cout << "Optimal solution for p = " << arg.at("p") << ":" << endl;
  //    cout << setw(30) << "Objective: " << res.at("f") << endl;
  //    cout << setw(30) << "Primal solution: " << res.at("x") << endl;
  //    cout << setw(30) << "Dual solution (x): " << res.at("lam_x") << endl;
  //    cout << setw(30) << "Dual solution (g): " << res.at("lam_g") << endl;
  //  }
  //  return 0;
}
