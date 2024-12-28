#ifndef WRITE_NUMBERS_HPP
#define WRITE_NUMBERS_HPP

#include <vector>
#include <utility>
#include <cmath>

class NumberCoordinatesProvider
{
public:
    NumberCoordinatesProvider(int max_grid_size);
    std::vector<std::pair<double, double>> getNumberCoordinates(int number) const;
    int getMaxGridSize() const;

private:
    int max_grid_size;
    void addSmoothCurve(std::vector<std::pair<double, double>> &coordinates, double x1, double y1, double x2, double y2, int segments) const;
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
std::vector<std::pair<double, double>> NumberCoordinatesProvider::getNumberCoordinates(int number) const
{
    std::vector<std::pair<double, double>> coordinates;
    std::vector<std::pair<double, double>> scaled_coordinates;
    double scale_factor = max_grid_size; // Assuming the grid size is 3x3

    switch (number)
    {
    case 0:
        coordinates = {
            {16.82, 7.98},
            {16.66, 8.02},
            {16.12, 8.36},
            {15.46, 8.69},
            {15.17, 8.69},
            {14.64, 8.69},
            {14.22, 8.56},
            {14.06, 8.32},
            {14.06, 8.15},
            {14.06, 7.53},
            {14.35, 7.49},
            {14.72, 7.37},
            {15.46, 7.41},
            {15.99, 7.74},
            {16.49, 8.32}};
        break;
    case 1:
        coordinates = {
            {13.58, 11.14},
            {14.11, 11.04},
            {14.80, 11.04},
            {15.09, 10.70},
            {14.85, 10.46},
            {14.18, 10.03},
            {13.55, 9.74},
            {13.02, 9.30},
            {12.39, 8.92},
            {12.10, 8.77},
            {11.76, 8.92},
            {11.33, 9.45},
            {11.33, 9.45},
            {11.72, 8.96},
            {12.29, 8.19},
            {12.49, 7.81},
            {12.58, 7.66}};
        break;
    case 2:
        coordinates = {
            {12.37, 12.59},
            {15.00, 9.58},
            {11.14, 10.05},
            {13.11, 7.85}};
        break;
    case 3:
        coordinates = {
            {16.99, 7.37},
            {16.81, 8.08},
            {16.56, 8.66},
            {16.28, 9.35},
            {15.45, 9.90},
            {15.04, 9.90},
            {14.63, 9.76},
            {14.63, 9.48},
            {14.81, 9.07},
            {15.04, 8.47},
            {15.18, 7.97},
            {15.32, 7.51},
            {15.50, 7.09},
            {14.72, 7.83},
            {14.53, 8.24},
            {14.12, 8.66},
            {13.75, 9.16},
            {12.88, 9.35},
            {12.70, 9.21},
            {12.60, 8.84},
            {12.74, 8.29},
            {13.02, 7.88},
            {13.16, 7.42},
            {13.62, 6.91},
            {13.75, 6.50}};
        break;
    case 4:
        coordinates = {
            {15.43, 7.39},
            {12.06, 11.44},
            {15.54, 10.43},
            {12.90, 8.29}};
        break;
    case 5:
        coordinates = {
            {16.59, 7.48},
            {13.59, 11.07},
            {12.20, 10.66},
            {14.46, 6.64},
            {12.93, 6.45},
            {10.56, 10.05}};
        break;
    case 6:
        coordinates = {
            {17.65, 5.78},
            {15.30, 9.45},
            {13.77, 9.04},
            {15.43, 5.39},
            {13.71, 5.12},
            {12.31, 8.40},
            {13.93, 9.04}};
                    break;

    case 7:
        coordinates = {
            {13.49, 4.73},
            {17.43, 5.25},
            {15.92, 7.24}};
        break;
    case 8:
        coordinates = {
            {13.13, 11.11},
            {15.07, 10.33},
            {14.39, 8.87},
            {12.61, 9.49},
            {10.78, 9.74},
            {11.32, 8.29},
            {12.44, 7.83},
            {13.02, 10.95}};
        break;

    case 9:
        coordinates = {
            {11.27, 8.32},
            {15.47, 10.74},
            {12.22, 14.05},
            {11.07, 12.23},
            {12.79, 9.85}};
        break;
    case 10:
        coordinates = {
            {11.42, 10.26},
            {15.27, 10.26},
            {15.95, 7.93},
            {13.21, 7.52},
            {11.56, 9.85}};
        break;
    case 11:
        coordinates = {
            {-7.23, 13.07},
            {15.21, 6.79},
            {5.48, -17.80}};
        break;
    case 12:
        coordinates = {
            {-2.24, 15.93},
            {3.82, 17.11},
            {8.69, 14.82},
            {12.56, 10.88},
            {15.07, 6.58}};
        break;
    default:
        return {};
        break;
    }

    for (auto &coord : coordinates)
    {
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
void NumberCoordinatesProvider::addSmoothCurve(std::vector<std::pair<double, double>> &coordinates, double x1, double y1, double x2, double y2, int segments) const
{
    for (int i = 0; i <= segments; ++i)
    {
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
int NumberCoordinatesProvider::getMaxGridSize() const
{
    return max_grid_size;
}

#endif // WRITE_NUMBERS_HPP
