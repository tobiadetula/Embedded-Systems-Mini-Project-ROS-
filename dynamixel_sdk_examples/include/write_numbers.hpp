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

/**
 * @brief Get the maximum grid size.
 * 
 * @return int The maximum grid size.
 */
int NumberCoordinatesProvider::getMaxGridSize() const {
    return max_grid_size;
}

#endif // WRITE_NUMBERS_HPP
