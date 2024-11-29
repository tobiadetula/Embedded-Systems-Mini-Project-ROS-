#ifndef INVERSE_KINEMATICS_HPP
#define INVERSE_KINEMATICS_HPP

#include <vector>
#include <cmath>
#include <functional>

class InverseKinematics {
public:
    InverseKinematics(double r1, double r2, double r3);
    ~InverseKinematics();

    // Method to calculate joint angles from end-effector position
    std::vector<double> calculateJointAngles(const std::vector<double>& endEffectorPosition);

private:
    // Add private methods and member variables as needed
    double linkLength1;
    double linkLength2;
    double linkLength3;
    double D;

    // Helper methods for calculations
    double calculateTheta1(double x, double y);
    double calculateTheta2(double x, double y);

    // Method to normalize dimensions
    void normalizeDimensions();
};

InverseKinematics::InverseKinematics(double r1, double r2, double r3) 
    : linkLength1(r1), linkLength2(r2), linkLength3(r3) {
    normalizeDimensions();
}

InverseKinematics::~InverseKinematics() {}

void InverseKinematics::normalizeDimensions() {
    D = (linkLength1 + linkLength2 + linkLength3) / 3.0;
    linkLength1 /= D;
    linkLength2 /= D;
    linkLength3 /= D;
}

std::vector<double> InverseKinematics::calculateJointAngles(const std::vector<double>& endEffectorPosition) {
    // Normalize end-effector position
    double x = endEffectorPosition[0] / D;
    double y = endEffectorPosition[1] / D;

    // Calculate joint angles
    double theta1 = calculateTheta1(x, y);
    double theta2 = calculateTheta2(x, y);

    return {theta1, theta2};
}

double InverseKinematics::calculateTheta1(double x, double y) {
    auto equation1 = [x, y, this](double theta) {
        return pow(x - linkLength1 * cos(theta) + linkLength3, 2) +
               pow(y - linkLength1 * sin(theta), 2) -
               pow(linkLength2, 2);
    };

    double initial_guess = 0.5; // Initial guess for theta1
    return solveNewtonRaphson(equation1, initial_guess);
}

double InverseKinematics::calculateTheta2(double x, double y) {
    auto equation2 = [x, y, this](double theta) {
        return pow(x - linkLength1 * cos(theta) - linkLength3, 2) +
               pow(y - linkLength1 * sin(theta), 2) -
               pow(linkLength2, 2);
    };

    double initial_guess = 0.5; // Initial guess for theta2
    return solveNewtonRaphson(equation2, initial_guess);
}

double InverseKinematics::solveNewtonRaphson(std::function<double(double)> func, double initial_guess, double tolerance, int max_iter) {
    double x = initial_guess;
    for (int i = 0; i < max_iter; i++) {
        double fx = func(x);
        double dfx = (func(x + tolerance) - func(x - tolerance)) / (2 * tolerance);
        double x_next = x - fx / dfx;
        if (fabs(x_next - x) < tolerance) {
            return x_next;
        }
        x = x_next;
    }
    throw std::runtime_error("Newton-Raphson did not converge!");
};

#endif // INVERSE_KINEMATICS_HPP