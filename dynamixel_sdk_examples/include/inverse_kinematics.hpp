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
    // Helper methods for calculations
    std::vector<double> calculateTheta(double x, double y);
};

InverseKinematics::InverseKinematics(double r1, double r2)
    : linkLength1(r1), linkLength2(r2)
{
}

InverseKinematics::~InverseKinematics() {}

std::vector<double> InverseKinematics::calculateJointAngles(const std::vector<double> &endEffectorPosition)
{
    double x = endEffectorPosition[0];
    double y = endEffectorPosition[1];

    try
    {
        // Calculate joint angles using inverse kinematics
        std::vector<double> thetas = calculateTheta(x, y);
        double theta1 = thetas[0];
        double theta2 = thetas[1];

        // Validate results before returning
        if (std::isnan(theta1) || std::isnan(theta2))
        {
            throw std::runtime_error("Theta calculation resulted in NaN.");
        }

        // Ensure angles are within 0 to 300 degrees
        // if (theta1 < 0) theta1 += 180;
        // if (theta2 < 0) theta2 += 180;
        // Wrap within 0 to 300 degrees if necessary
        theta1 = fmod(theta1, 300); 
        theta2 = fmod(theta2, 300);

        theta1 = theta1 + 90; // Adjust for motor offset
        theta2 = theta2 + 90; // Adjust for motor offset
        return {theta1, theta2};
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error calculating joint angles: " << e.what() << std::endl;
        return {-1.0, -1.0}; // Return error value for invalid points
    }
}

std::vector<double> InverseKinematics::calculateTheta(double x, double y)
{
    double theta1, theta2;

    // Calculate theta2 using law of cosines
    double cosTheta2 = (pow(x, 2) + pow(y, 2) - pow(linkLength1, 2) - pow(linkLength2, 2)) / (2 * linkLength1 * linkLength2);

    // Ensure the cosTheta2 value is within the valid range [-1, 1]
    if (cosTheta2 < -1.0 || cosTheta2 > 1.0) {
        std::cerr << "Point out of reach!\n";
        return {std::nan(""), std::nan("")}; // Return NaN to indicate error
    }

    // Calculate theta2
    theta2 = acos(cosTheta2);

    // Calculate theta1 using geometry
    theta1 = atan2(y, x) - atan2(linkLength2 * sin(theta2), linkLength1 + linkLength2 * cos(theta2));

    // Convert radians to degrees
    theta1 = theta1 * 180.0 / M_PI;
    theta2 = theta2 * 180.0 / M_PI;

    // Return calculated joint angles in degrees
    return {theta1, theta2};
}

#endif // INVERSE_KINEMATICS_HPP
