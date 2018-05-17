#pragma once

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  vector<double> Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, const int N, const double dt, const double ref_v_meterspersecond);

private:

	

};

