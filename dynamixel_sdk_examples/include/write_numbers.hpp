#ifndef WRITE_NUMBERS_HPP
#define WRITE_NUMBERS_HPP

#include <vector>
#include <utility>

class NumberCoordinatesProvider {
public:
    NumberCoordinatesProvider(int max_grid_size);
    std::vector<std::pair<double, double>> getNumberCoordinates(int number) const;
    int getMaxGridSize() const;

private:
    int max_grid_size;
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
    double scale_factor = max_grid_size / 3.0; // Assuming the grid size is 3x3

    switch (number) {
        case 0:
            coordinates = {{1, 1}, {2, 1}, {2, 2}, {1, 2}, {1, 1}};
            break;
        case 1:
            coordinates = {{2, 1}, {2, 3}};
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
            coordinates = {{2, 3}, {1, 3}, {1, 1}, {2, 1}, {2, 2}, {1, 2}};
            break;
        case 7:
            coordinates = {{1, 3}, {2, 3}, {2, 2}, {2, 1}};
            break;
        case 8:
            coordinates = {{1, 1}, {2, 1}, {2, 3}, {1, 3}, {1, 1}, {1, 2}, {2, 2}};
            break;
        case 9:
            coordinates = {{2, 1}, {1, 1}, {1, 3}, {2, 3}, {2, 2}, {1, 2}};
            break;
        default:
        coordinates = {{1, 1}, {max_grid_size, max_grid_size}};
            return {};
    }

    for (auto& coord : coordinates) {
        double x = coord.first * scale_factor;
        double y = coord.second * scale_factor;
        scaled_coordinates.push_back({x, y});
    }

    return scaled_coordinates;
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
