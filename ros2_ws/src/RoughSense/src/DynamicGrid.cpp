/**
 * @file DynamicGrid.cpp
 * @author XX.XX@uni.XX
 * @version 0.1
 * @date 2025-02-04
 * 
 * @copyright XX XX | 2024
 * @brief Implementation file for Class DynamicGrid.
 * @details This is a C++ DynamicGrid library taking a PCL point cloud as an input and returning an image. CalculateRoughness is the core method of this library.
 */

#include "DynamicGrid.hpp"

// A dynamic grid class that supports negative world coordinates.


// Get the value at a world coordinate (r, c)
int DynamicGlobalGrid::get(int r, int c, int indx) const {
    auto [i, j] = worldToIndex(r, c);
    if (i >= grid.size() || j >= grid[0].size()) {
        throw std::out_of_range("Coordinates out of range");
    }
    return grid[i][j][indx];
};

// Place a local grid into the global grid at the specified world coordinate (rowOffset, colOffset)
// The local grid is provided as a vector of vectors.
void DynamicGlobalGrid::placeLocalGrid(const TerrainGrid &localGrid, int rowOffset, int colOffset) {
    // Determine the world coordinate extents of the local grid.
    int localRows = localGrid.size();
    if (localRows == 0) return;
    int localCols = localGrid[0].size();
    
    // Determine the world coordinate bounds of the local grid.
    int worldTop = rowOffset;
    int worldLeft = colOffset;
    int worldBottom = rowOffset + localRows - 1;
    int worldRight = colOffset + localCols - 1;

    // Ensure the global grid is large enough to cover the new region.
    ensureSizeForWorldCoordinates(worldTop, worldLeft, worldBottom, worldRight);

    // Place the local grid's values into the global grid.
    for (int i = 0; i < localRows; ++i) {
        for (int j = 0; j < localCols; ++j) {
            auto [globalRow, globalCol] = worldToIndex(rowOffset + i, colOffset + j);
            grid[globalRow][globalCol] = localGrid[i][j];
        }
    }
};

// For debugging: print the current global grid (world coordinates relative to the internal origin)
void DynamicGlobalGrid::print() const {
    std::cout << "Global grid (each cell holds 3 values):\n";
        std::cout << "Internal origin (world (0,0) is at grid index): (" 
                  << originRow << ", " << originCol << ")\n";
        for (size_t i = 0; i < grid.size(); ++i) {
            for (size_t j = 0; j < grid[i].size(); ++j) {
                // Print the world coordinates for this cell.
                int worldRow = static_cast<int>(i) - originRow;
                int worldCol = static_cast<int>(j) - originCol;
                std::cout << "(" << worldRow << "," << worldCol << "): [";
                for (double d : grid[i][j]) {
                    std::cout << d << " ";
                }
                std::cout << "]\t";
            }
            std::cout << "\n";
        }
        std::cout << "---------------------------\n";
};



// Convert world coordinates (which can be negative) into indices into the grid.
pair<size_t, size_t> DynamicGlobalGrid::worldToIndex(int worldRow, int worldCol) const {
    int i = worldRow + originRow;
    int j = worldCol + originCol;
    if (i < 0 || j < 0) {
        throw std::out_of_range("Computed negative index; grid is not resized properly.");
    }
    return {static_cast<size_t>(i), static_cast<size_t>(j)};
};

// Ensure that the grid covers world coordinates from (worldTop, worldLeft) to (worldBottom, worldRight).
void DynamicGlobalGrid::ensureSizeForWorldCoordinates(int worldTop, int worldLeft, int worldBottom, int worldRight) {
     // If the grid is empty, initialize it with a minimal size.
        if (grid.empty()) 
        {
            // Determine a minimal size based on positive coordinates.
            int neededRows = max(worldBottom, 0) + 1;
            int neededCols = max(worldRight, 0) + 1;
            grid.resize(neededRows, vector<TerrainCell>(neededCols, TerrainCell(3, 0.0)));
            originRow = 0;
            originCol = 0;
        }

        // Current grid dimensions.
        int currentRows = grid.size();
        int currentCols = grid[0].size();

        // Calculate current world coordinate bounds.
        int currentWorldTop = -originRow;
        int currentWorldBottom = currentRows - originRow - 1;

        // Determine how many rows to add on the top (if needed).
        int addTop = (worldTop < currentWorldTop) ? (currentWorldTop - worldTop) : 0;
        // And on the bottom.
        int addBottom = (worldBottom > currentWorldBottom) ? (worldBottom - currentWorldBottom) : 0;

        // Insert rows at the top.
        for (int i = 0; i < addTop; ++i) {
            vector<TerrainCell> newRow(currentCols, TerrainCell(3, 0.0));
            grid.insert(grid.begin(), newRow);
        }
        originRow += addTop;  // Adjust the origin.
        currentRows = grid.size();  // Update current row count.

        // Append rows at the bottom.
        for (int i = 0; i < addBottom; ++i) {
            vector<TerrainCell> newRow(currentCols, TerrainCell(3, 0.0));
            grid.push_back(newRow);
        }

        // Now adjust columns.
        currentCols = grid[0].size();
        int currentWorldLeft = -originCol;
        int currentWorldRight = currentCols - originCol - 1;
        int addLeft = (worldLeft < currentWorldLeft) ? (currentWorldLeft - worldLeft) : 0;
        int addRight = (worldRight > currentWorldRight) ? (worldRight - currentWorldRight) : 0;

        // Insert columns on the left for every row.
        if (addLeft > 0) {
            for (auto &row : grid) {
                row.insert(row.begin(), addLeft, TerrainCell(3, 0.0));
            }
            originCol += addLeft;
        }
        // Append columns on the right for every row.
        if (addRight > 0) {
            for (auto &row : grid) {
                row.insert(row.end(), addRight, TerrainCell(3, 0.0));
            }
        }
};
