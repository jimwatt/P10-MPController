#include <iostream>

#include "utilities.hpp"

#include "Eigen-3.3/Eigen/QR"

// Transform vector of x, y points in global frame to vehicle frame
std::pair<Eigen::VectorXd,Eigen::VectorXd> global2veh(const double veh_x, const double veh_y, const double psi, 
                            const Eigen::VectorXd& x, const Eigen::VectorXd& y) {
  
  assert(x.size()==y.size());
  const int numpts = x.size();

  Eigen::MatrixXd xy = Eigen::MatrixXd::Zero(2,numpts);
  xy.row(0) = x.array() - veh_x;
  xy.row(1) = y.array() - veh_y;

  Eigen::Matrix2d R;
  R(0,0) = cos(psi);
  R(0,1) = sin(psi);
  R(1,0) = -sin(psi);
  R(1,1) = cos(psi);

  Eigen::MatrixXd tXY = R * xy;

  return std::make_pair(tXY.row(0),tXY.row(1));

}

// Evaluate a polynomial.
double polyeval(const Eigen::VectorXd& coeffs, const double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals,
                        const int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

double mph2mps(const double v_mph) {
  return v_mph * 1609.34 / 3600.0;
}

// For converting back and forth between radians and degrees.
double deg2rad(const double x) { return x * M_PI / 180.0; }
double rad2deg(const double x) { return x * 180.0 / M_PI; }


