#pragma once

#include <iostream>

#include "Eigen-3.3/Eigen/Core"

// Transform vector of x, y points in global frame to vehicle frame
std::pair<Eigen::VectorXd,Eigen::VectorXd> global2veh(const double veh_x, const double veh_y, const double psi, 
                            const Eigen::VectorXd& x, const Eigen::VectorXd& y);

// Evaluate a polynomial.
double polyeval(const Eigen::VectorXd& coeffs, const double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, const int order);


double mph2mps(const double v_mph);

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(const double x);
double rad2deg(const double x);


template <typename T>
void print_vector(const std::vector<T>& v) {
  for(size_t ii=0;ii<v.size();++ii) {
    std::cout << v[ii] << " ";
  }
  std::cout << std::endl;
}