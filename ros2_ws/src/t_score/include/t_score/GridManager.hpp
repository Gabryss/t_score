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
#include <algorithm>
#include <pcl/point_types.h>



using namespace std;


// Each cell is a structure of double and integer representing terrain state.
struct TerrainCell
{
    std::vector<pcl::PointXYZ> points; // optional, if you still need raw points
    double roughness     = 0.0;  // std-dev of distances to plane
    double slope         = 0.0;  // slope (rad)
    double height        = 0.0;  // step height (m)
    double mean_z        = 0.0;  // mean z in this cell
    bool traversable     = true;
    int    num_points    = 0;

};

// Structure to handle grid coordinates
struct GridCoord {
    int x;
    int y;
};


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
        int offset_static = 0;

        // ===========================
        // Methods
        // ===========================
        TerrainCell get_cell(TerrainGrid& grid, int r, int c, int indx) const;
        void create_grid(TerrainGrid& grid, int size_m_x, int size_m_y, float res);
        void create_local_global_grids(int global_map_size, int local_map_size, float res);
        // void update_map(int rover_x, int rover_y);
        GridCoord pose_to_grid_coordinates(double x, double y);
        void compute_step_heights_local(int window_radius_cells);
    
        protected:
        // ===========================
        // Attributes
        // ===========================
        

        // ===========================
        // Methods
        // ===========================
        pair<size_t, size_t> worldToIndex(int worldRow, int worldCol) const;
};


