#ifndef WRITE_NUMBERS_HPP
#define WRITE_NUMBERS_HPP

#include <vector>
#include "inverse_kinematics.hpp"

class NumberWriter {
public:
    NumberWriter(int max_grid_size);
    void writeNumber(int number);

private:
    int max_grid_size;
    InverseKinematics ik;

    std::vector<std::pair<double, double>> getNumberCoordinates(int number);
    void moveToCoordinates(const std::vector<std::pair<double, double>>& coordinates);
};

/**
 * @brief Get the coordinates for drawing a given number.
 * 
 * This function returns a vector of pairs of doubles, where each pair represents
 * the (x, y) coordinates of points that can be used to draw the specified number.
 * The coordinates are designed to be used in a 2D plane for drawing purposes.
 * 
 * @param number The number to get the coordinates for. Valid values are 0-9.
 * @return std::vector<std::pair<double, double>> A vector of coordinate pairs for drawing the number.
 *         If the number is not in the range 0-9, an empty vector is returned.
 */
std::vector<std::pair<double, double>> NumberWriter::getNumberCoordinates(int number) {
    switch (number) {
        case 0:
            return {{1, 1}, {2, 1}, {2, 2}, {1, 2}, {1, 1}};
        case 1:
            return {{2, 1}, {2, 3}};
        case 2:
            return {{1, 3}, {2, 3}, {2, 2}, {1, 2}, {1, 1}, {2, 1}};
        case 3:
            return {{1, 3}, {2, 3}, {2, 2}, {1, 2}, {2, 2}, {2, 1}, {1, 1}};
        case 4:
            return {{1, 3}, {1, 2}, {2, 2}, {2, 3}, {2, 1}};
        case 5:
            return {{2, 3}, {1, 3}, {1, 2}, {2, 2}, {2, 1}, {1, 1}};
        case 6:
            return {{2, 3}, {1, 3}, {1, 1}, {2, 1}, {2, 2}, {1, 2}};
        case 7:
            return {{1, 3}, {2, 3}, {2, 2}, {2, 1}};
        case 8:
            return {{1, 1}, {2, 1}, {2, 3}, {1, 3}, {1, 1}, {1, 2}, {2, 2}};
        case 9:
            return {{2, 1}, {1, 1}, {1, 3}, {2, 3}, {2, 2}, {1, 2}};
        default:
            return {};
    }
}

#endif // WRITE_NUMBERS_HPP
