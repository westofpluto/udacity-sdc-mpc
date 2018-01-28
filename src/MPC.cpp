#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;
using CppAD::Value;

// Set the timestep length and duration
size_t N = 30;
double dt = 0.05;
size_t num_state_vars = 6;
size_t num_control_vars = 2;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// NOTE: feel free to play around with this
// or do something completely different
double ref_v = 80;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

//
// create some penalty constants for the cost
//
double P_cte = 1000.0;
double P_epsi = 40000.0;
//double P_v = max(1.0, 20.0 - (ref_v - 70.0)/2.0);
double P_v = 10.0;
double P_vdelta = 2500.0;
double P_delta = 50.0;
double P_a = 1.0;
double P_delta_change = 60.0;
double P_a_change = 1.0;
double P_curve = 0.0;
//
// Class FG_eval does what it says: It has a single function call operator () that takes vector of vars and evaluates
// everything in the ADVector& fg, which includes the cost function and all teh constraints
//
class FG_eval {
   public:
      // Fitted polynomial coefficients
      Eigen::VectorXd coeffs;
      FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

      typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

      void operator()(ADvector& fg, const ADvector& vars) {
          //
          // The cost is stored is the first element of fg
          //
          fg[0] = 0;

          //
          // Cost function: Note that we must multiply the individual costs by constants to penalize all errors appropriately
          //

          // The part of the cost based on the reference state.
          for (int t = 0; t < N; t++) {
              fg[0] += P_cte*CppAD::pow(vars[cte_start + t], 2);
              fg[0] += P_epsi*CppAD::pow(vars[epsi_start + t], 2);
              fg[0] += P_v*CppAD::pow(vars[v_start + t] - ref_v, 2);
          }

          //
          // penalize speed for high curvature of reference trajectory
          // 
          for (int t = 0; t < N; t++) {
              AD<double> xt=vars[x_start + t];
              AD<double> fderiv = coeffs[1] + 2.0*coeffs[2] * xt + 3.0*coeffs[3]*xt*xt;
              AD<double> curvature = CppAD::fabs(CppAD::atan(fderiv));
              fg[0] += P_curve*curvature*CppAD::pow(vars[v_start + t], 2);
          }

          // Minimize the use of actuators.
          for (int t = 0; t < N - 1; t++) {
            fg[0] += P_delta*CppAD::pow(vars[delta_start + t], 2);
            fg[0] += P_a*CppAD::pow(vars[a_start + t], 2);
            // important: penalize delta according to speed
            fg[0] += P_vdelta*CppAD::pow(vars[delta_start + t] * vars[v_start+t], 2);
          }

          // Minimize the value gap between sequential actuations.
          for (int t = 0; t < N - 2; t++) {
            fg[0] += P_delta_change*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += P_a_change*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
          }

          std::cout << "TEST: cost " << fg[0] << std::endl;

          //
          // set up our constraint equations
          //

          // initial state constraints
          fg[1 + x_start] = vars[x_start];
          fg[1 + y_start] = vars[y_start];
          fg[1 + psi_start] = vars[psi_start];
          fg[1 + v_start] = vars[v_start];
          fg[1 + cte_start] = vars[cte_start];
          fg[1 + epsi_start] = vars[epsi_start];

          // trajectory (t>0) state constraints
          for (int t = 1; t < N; t++) {
              // The state at time t+1 .
              AD<double> x1 = vars[x_start + t];
              AD<double> y1 = vars[y_start + t];
              AD<double> psi1 = vars[psi_start + t];
              AD<double> v1 = vars[v_start + t];
              AD<double> cte1 = vars[cte_start + t];
              AD<double> epsi1 = vars[epsi_start + t];

              // The state at time t.
              AD<double> x0 = vars[x_start + t - 1];
              AD<double> y0 = vars[y_start + t - 1];
              AD<double> psi0 = vars[psi_start + t - 1];
              AD<double> v0 = vars[v_start + t - 1];
              AD<double> cte0 = vars[cte_start + t - 1];
              AD<double> epsi0 = vars[epsi_start + t - 1];

              // Only consider the actuation at time t.
              AD<double> delta0 = vars[delta_start + t - 1];
              AD<double> a0 = vars[a_start + t - 1];

              //
              // trajectory errors from polyfit trajectory (3rd order)
              //
              AD<double> f0 = coeffs[0] + x0*(coeffs[1] + x0*(coeffs[2] + x0*coeffs[3]));
              AD<double> psides0 = CppAD::atan(coeffs[1] + x0*(2.0*coeffs[2] + x0*3.0*coeffs[3]));

              //
              // Recall the equations for the model from the video lectures. However, in the simulator delta is the opposite of the delta
              // that we have used in our equations in the videos, so we have to modify the equations of motion that depend on delta.
              // So, we change signs in the equations for psi and epsi
              //
              // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
              // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
              // psi_[t] = psi[t-1] - (v[t-1] / Lf) * delta[t-1] * dt
              // v_[t] = v[t-1] + a[t-1] * dt
              // cte[t] = y[t - 1] - f(x[t-1]) + v[t-1] * sin(epsi[t-1]) * dt
              // epsi[t] = psi[t] - psides[t-1] - v[t-1] * delta[t-1] / Lf * dt

              fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
              fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
              fg[1 + psi_start + t] = psi1 - (psi0 - (v0/Lf) * delta0 * dt);
              fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
              fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
              fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - (v0/Lf) * delta0 * dt);
          }
      }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    //
    // Set the number of model variables (includes both states and inputs).
    //
    size_t n_vars = num_state_vars*N + num_control_vars*(N-1);
    size_t n_constraints = num_state_vars * N;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    //
    // lower and upper bounds for state variables = +/- large numbers (allow them to have basically any value)
    //
    for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    //
    // lower and upper limits of control variables are specified in the problem statement
    //
    // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
    for (int i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Acceleration/decceleration upper and lower limits.
    for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    //
    // Lower and upper limits for the constraints.
    // For all states except the initial state, constrainst should be 0.
    //
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    //
    // Handle the initial state constraints, which are that the current values of the constraints must be 
    // equal to what is specified in input state
    //
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          100.0\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    if (ok) {
        std::cout << "OK = TRUE" << std::endl;
    } else {
        std::cout << "OK = FALSE" << std::endl;
    }

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    // Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.
    //
    // Return starting (delta,a)
    //

    vector<double> optvars;
    optvars.push_back(solution.x[delta_start]);
    optvars.push_back(solution.x[a_start]);

    for (int i = 0; i < N-1; i++) {
        optvars.push_back(solution.x[x_start + i + 1]);
        optvars.push_back(solution.x[y_start + i + 1]);
    }
    return optvars;
}

double MPC::get_dt() {
    return dt;
}
double MPC::get_Lf() {
    return Lf;
}

//
// polyeval: Evaluate a polynomial.
//
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}
//
// polyderiveval: Evaluate the derivative of the polynomial.
//
double polyderiveval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 1; i < coeffs.size(); i++) {
        result += coeffs[i] * i * pow(x, i-1);
    }
    return result;
}


