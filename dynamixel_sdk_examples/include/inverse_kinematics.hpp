#ifndef INVERSE_KINEMATICS_HPP
#define INVERSE_KINEMATICS_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <algorithm> // For std::clamp

class InverseKinematics
{
public:
    InverseKinematics(double r1, double r2);
    ~InverseKinematics();

    // Method to calculate joint angles from end-effector position
    std::vector<double> calculateJointAngles(const std::vector<double> &endEffectorPosition);

private:
    double linkLength1;
    double linkLength2;
    double D;
    // Helper methods for calculations
    std::vector<double> calculateTheta(double x, double y);


    // Method to normalize dimensions
    void normalizeDimensions();
};

InverseKinematics::InverseKinematics(double r1, double r2)
    : linkLength1(r1), linkLength2(r2)
{
}

InverseKinematics::~InverseKinematics() {}

void InverseKinematics::normalizeDimensions()
{
    // This method is now empty as dimension normalization is removed
}

std::vector<double> InverseKinematics::calculateJointAngles(const std::vector<double> &endEffectorPosition)
{
    // double x = endEffectorPosition[0] / D;
    // double y = endEffectorPosition[1] / D;

    double x = endEffectorPosition[0];
    double y = endEffectorPosition[1];

    // Set initial guesses dynamically or adjust as needed
    // double initial_guess_theta1_deg = 90.0; // Adjust if necessary
    // double initial_guess_theta2_deg = 90.0;

    try
    {
        // Calculate joint angles using Newton-Raphson
        std::vector<double> thetas = calculateTheta(x, y);
        double theta1 = thetas[0];
        double theta2 = thetas[1];
        // Validate results before clamping
        if (std::isnan(theta1) || std::isnan(theta2))
        {
            throw std::runtime_error("Theta calculation resulted in NaN.");
        }

        // Avoid aggressive clamping unless necessary
        // theta1 = 300 - theta1; // Adjust for motor orientation


        return {theta1, theta2};
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error calculating joint angles: " << e.what() << std::endl;
        return {300.0, 300.0}; // Return default or error value
    }
}

std::vector<double>  InverseKinematics::calculateTheta(double x, double y)
{

    double theta1;
    double theta2;

    // Calculate theta2
    double cosTheta2 = (pow(x, 2) + pow(y, 2) - pow(linkLength1, 2) - pow(linkLength2, 2)) / (2 * linkLength1 * linkLength2);
    if (cosTheta2 < -1.0 || cosTheta2 > 1.0) {
        std::cerr << "Point out of reach!\n";
        return {300.0, 300.0};
    }
    theta2 = acos(cosTheta2);
            // theta1 = theta1 + 120; // Adjust for motor 
    // theta2 = theta2 - (M_PI*2/3); // Adjust for motor orientation

    // Calculate theta1
    theta1 = atan2(y, x) - atan2(linkLength2 * sin(theta2), linkLength1 + linkLength2 * cos(theta2));

    // Convert to degrees
    theta1 = theta1 * 180.0 / M_PI;
    theta2 = theta2 * 180.0 / M_PI;

    // Ensure angles are within 0 to 300 degrees
    if (theta1 < 0) theta1 += 360;
    if (theta2 < 0) theta2 += 360;

    theta1 = fmod(theta1, 300); // Wrap within 0 to 300
    theta2 = fmod(theta2, 300);


    return {theta1, theta2};    
};

#endif // INVERSE_KINEMATICS_HPP