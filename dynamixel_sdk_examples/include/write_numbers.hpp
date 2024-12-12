#ifndef WRITE_NUMBERS_HPP
#define WRITE_NUMBERS_HPP

#include <vector>
#include <utility>
#include <cmath>

class NumberCoordinatesProvider {
public:
    NumberCoordinatesProvider(int max_grid_size);
    std::vector<std::pair<double, double>> getNumberCoordinates(int number) const;
    int getMaxGridSize() const;

private:
    int max_grid_size;
    void addSmoothCurve(std::vector<std::pair<double, double>>& coordinates, double x1, double y1, double x2, double y2, int segments) const;
};

/**
 * @brief Constructor for NumberCoordinatesProvider.
 * 
 * @param max_grid_size The maximum size of the grid within which numbers can be written.
 */
NumberCoordinatesProvider::NumberCoordinatesProvider(int max_grid_size) : max_grid_size(max_grid_size) {}

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
std::vector<std::pair<double, double>> NumberCoordinatesProvider::getNumberCoordinates(int number) const {
    std::vector<std::pair<double, double>> coordinates;
    std::vector<std::pair<double, double>> scaled_coordinates;
    double scale_factor = max_grid_size; // Assuming the grid size is 3x3

    switch (number) {
        case 0:
            coordinates = {
                {1.800, 1.500}, {1.742, 1.795}, {1.591, 1.976}, {1.405, 1.974}, 
                {1.255, 1.789}, {1.200, 1.492}, {1.261, 1.198}, {1.414, 1.021}, 
                {1.600, 1.029}, {1.748, 1.218}
            };
            break;
        case 1:
            coordinates = {{1, 1}, {1, 2}};
            break;
        case 2:
            coordinates = {{1, 3}, {2, 3}, {2, 2}, {1, 2}, {1, 1}, {2, 1}};
            break;
        case 3:
            coordinates = {{1, 3}, {2, 3}, {2, 2}, {1, 2}, {2, 2}, {2, 1}, {1, 1}};
            break;
        case 4:
            coordinates = {{1, 3}, {1, 2}, {2, 2}, {2, 3}, {2, 1}};
            break;
        case 5:
            coordinates = {{2, 3}, {1, 3}, {1, 2}, {2, 2}, {2, 1}, {1, 1}};
            break;
        case 6:
            coordinates = {{2, 3}, {1, 3}, {1, 2}, {2, 2}, {2, 1}, {1, 1}, {1, 2}};
            break;
        case 7:
            coordinates = {{1, 3}, {2, 3}, {2, 2}, {2, 1}};
            break;
        case 8:
            coordinates = {{1, 3}, {2, 3}, {2, 2}, {1, 2}, {1, 1}, {2, 1}, {1, 2}, {2, 2}};
            break;
        case 9:
            coordinates = {{1, 1}, {2, 1}, {2, 2}, {1, 2}, {1, 3}, {2, 3}, {2, 2}};
            break;
        case 10:
            coordinates = {
                {11.42, 10.26}, 
                {15.27, 10.26}, 
                {15.95, 7.93}, 
                {13.21, 7.52}, 
                {11.56, 9.85}
            };
            break;
        case 11:
            coordinates = {
                {-7.23, 13.07}, 
                {15.21, 6.79}, 
                {5.48, -17.80}
            };
            break;
        case 12:
            coordinates = {
                {-2.24, 15.93}, 
                {3.82, 17.11}, 
                {8.69, 14.82}, 
                {12.56, 10.88}, 
                {15.07, 6.58}
            };
            break;
        default:
            return {};
            break;
    }

    for (auto& coord : coordinates) {
        double x = coord.first * scale_factor;
        double y = coord.second * scale_factor;
        scaled_coordinates.push_back({x, y});
    }

    return scaled_coordinates;
}

/**
 * @brief Add smooth curve points between two coordinates.
 * 
 * @param coordinates The vector to add the points to.
 * @param x1 The starting x coordinate.
 * @param y1 The starting y coordinate.
 * @param x2 The ending x coordinate.
 * @param y2 The ending y coordinate.
 * @param segments The number of segments to divide the curve into.
 */
void NumberCoordinatesProvider::addSmoothCurve(std::vector<std::pair<double, double>>& coordinates, double x1, double y1, double x2, double y2, int segments) const {
    for (int i = 0; i <= segments; ++i) {
        double t = static_cast<double>(i) / segments;
        double x = x1 + t * (x2 - x1);
        double y = y1 + t * (y2 - y1);
        coordinates.push_back({x, y});
    }
}

/**
 * @brief Get the maximum grid size.
 * 
 * @return int The maximum grid size.
 */
int NumberCoordinatesProvider::getMaxGridSize() const {
    return max_grid_size;
}

#endif // WRITE_NUMBERS_HPP
