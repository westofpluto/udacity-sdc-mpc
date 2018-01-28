#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

double polyeval(Eigen::VectorXd coeffs, double x); 
double polyderiveval(Eigen::VectorXd coeffs, double x); 

class MPC {
    public:
        MPC();

        virtual ~MPC();

        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuatotions.

        vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);


        double get_dt();
        double get_Lf();
};

#endif /* MPC_H */
