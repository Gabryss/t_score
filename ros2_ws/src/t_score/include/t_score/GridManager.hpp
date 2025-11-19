/**
 * @file GridManager.hpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2025-11-19
 * 
 * @copyright Gabriel Garcia | 2025
 * @brief Implementation file for Class GridManager.
 * @details This is a grid manager to handle terrain data.
 */


#pragma once

#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>


using namespace std;


// Each cell is a vector<double> of size 2 representing terrain state.
using TerrainCell = vector<double>;  
// The global grid is a 2D grid where each cell is a TerrainCell.
using TerrainGrid = vector<vector<TerrainCell>>;


class GridManager 
{
    public:
        GridManager(/* args */) {};
        ~GridManager() {};

        // ===========================
        // Attributes
        // ===========================
        TerrainGrid global_grid;
        TerrainGrid local_grid;
        float resolution; // Resolution of the cells (cm)
        int num_cells_x = 0;  // Number of cells in the x direction.
        int num_cells_y = 0;  // Number of cells in the y direction.
        int originRow = 0;  // The row in 'grid' that corresponds to world row 0.
        int originCol = 0;  // The column in 'grid' that corresponds to world col 0.
        
        // ===========================
        // Methods
        // ===========================
        double get(TerrainGrid grid, int r, int c, int indx) const;
        void create_grid(TerrainGrid grid, int size_m_x, int size_m_y, float res);
        void create_local_global_grids(int global_map_size, int local_map_size, float res);
        void update_map(int offset_x, int offset_y);
    
        protected:
        // ===========================
        // Attributes
        // ===========================
        

        // ===========================
        // Methods
        // ===========================
        pair<size_t, size_t> worldToIndex(int worldRow, int worldCol) const;
};


