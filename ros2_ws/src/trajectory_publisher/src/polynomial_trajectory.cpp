#include <trajectory_publisher/polynomial_trajectory.hpp>
#include <cmath>

PolynomialTrajectory::PolynomialTrajectory() {}

void PolynomialTrajectory::setTrajectory(double start, double goal, double T) {
    T_ = T;

    double D = goal - start;

    a0_ = start;
    a1_ = 0.0;
    a2_ = 0.0;

    a3_ = 10 * D / pow(T, 3);
    a4_ = -15 * D / pow(T, 4);
    a5_ = 6 * D / pow(T, 5);
}

double PolynomialTrajectory::getPosition(double t) {
    if (t > T_)
        t = T_;

    return a0_
        + a3_*pow(t, 3)
        + a4_*pow(t, 4)
        + a5_*pow(t, 5);
}