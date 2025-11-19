/**
 * @file MapCreator.cpp
 * @author XX.XX@uni.XX
 * @version 0.1
 * @date 2024-09-20
 * 
 * @copyright XX XX | 2024
 * @brief Implementation file for Class MapCreator.
 * @details This is a C++ library to create images based on a grid of vectors. Output an image
 */

#include "MapCreator.hpp"


// =====================================================
// Create traversability map
// =====================================================
Mat MapCreator::MakeMap(vector<vector<vector<double>>>& input_grid, int input_depth)
{
    Grid = input_grid;
    depth = input_depth;

    rows = Grid.size();              // Number of rows in the grid (5)
    cols = Grid[0].size();           // Number of columns in the grid (5)

    // Create an OpenCV matrix (image) to hold the 2D slice data
    std::vector< int > shape = {rows, cols};
    // image.reshape(CV_8UC1, shape);
    Mat new_image(rows, cols, CV_8UC1);
    image = new_image;
    
    // Fill the image based on the values of the grid
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            float value = Grid[i][j][depth];
            image.at<int8_t>(i, j) = value;
        }
    }

    return image;
};




// =====================================================
// TOOLS
// =====================================================
void MapCreator::saveMap(string path, Mat image)
{
    // Save the image
    imwrite(path, image);
};
