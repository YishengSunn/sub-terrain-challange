#ifndef POLYNOMIAL_TRAJECTORY_HPP
#define POLYNOMIAL_TRAJECTORY_HPP

class PolynomialTrajectory
{
public:
    PolynomialTrajectory();

    void setTrajectory(
        double start,
        double goal,
        double T);

    double getPosition(double t);

private:
    double a0_, a1_, a2_, a3_, a4_, a5_;
    double T_;
};

#endif