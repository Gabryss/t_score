/**
 * @file GridManager.cpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2025-11-19
 * 
 * @copyright Gabriel Garcia | 2025
 * @brief Implementation file for Class GridManager.
 * @details This is a grid manager to handle terrain data.
 */


#include "GridManager.hpp"



// Create local and global grids
void GridManager::create_local_global_grids(int global_map_size, int local_map_size, float res)
{
    // Create global grid
    create_grid(global_grid, global_map_size, global_map_size, res);
    // Create local grid
    create_grid(local_grid, local_map_size, local_map_size, res);

    // =====================================================
    // Compute static offset (between global and local)
    // =====================================================
    int global_origin_index = global_grid.size() / 2;
    int local_origin_index = local_grid.size() / 2;
    offset_static = (global_origin_index - local_origin_index);
    resolution = res;
};


// Create a grid
void GridManager::create_grid(TerrainGrid& grid, int size_m_x, int size_m_y, float res)
{
    grid.clear();
    num_cells_x = static_cast<int>(std::ceil(size_m_x / res));
    num_cells_y = static_cast<int>(std::ceil(size_m_y / res));

    // force odd sizes if you want perfect centering:
    if (num_cells_x % 2 == 0) num_cells_x++;
    if (num_cells_y % 2 == 0) num_cells_y++;

    // row-major: grid[y][x]
    grid.resize(num_cells_y);
    for (int y = 0; y < num_cells_y; ++y)
    {
        grid[y].resize(num_cells_x);
        for (int x = 0; x < num_cells_x; ++x)
        {
            grid[y][x] = TerrainCell{};  // value-init
        }
    }
}




void GridManager::update_map(int rover_x, int rover_y)
{
  if (global_grid.empty() || global_grid[0].empty()) 
  {
    std::cerr << "Error: Global grid not initialized." << std::endl;
    return; // grid not initialized
  }

  // Bounds of the global grid
  const int height = static_cast<int>(global_grid.size());
  const int width  = static_cast<int>(global_grid[0].size());

  const int half_local_map = local_grid.size()/2; // in meters

  // Iterate over a square window centered on the rover
  for (int dy = -half_local_map; dy < half_local_map; ++dy) {
      for (int dx = -half_local_map; dx < half_local_map; ++dx) {
          int x = rover_x + dx;
          int y = rover_y + dy;


          // Bounds check
          if (x < 0 || y < 0 || x >= width || y >= height) {
              continue;
          }
          int ly = dy + half_local_map;  // 0..19
          int lx = dx + half_local_map;  // 0..19

          // --- Update the global grid with the values stored in the local grid ---
          global_grid[y][x].roughness = local_grid[ly][lx].roughness;
          global_grid[y][x].slope = local_grid[ly][lx].slope;
          global_grid[y][x].height = local_grid[ly][lx].height;
          // -------------------------------------------------------------------------------  

      }
  }

};


GridCoord GridManager::pose_to_grid_coordinates(double x, double y)
{
    GridCoord out;

    if (global_grid.empty() || global_grid[0].empty()) 
    {
      // handle error: grid not initialized
      std::cerr << "Error: Global grid not initialized." << std::endl;
      return out;
    }

    int out_x = static_cast<int>(x/ resolution) + offset_static;
    int out_y = static_cast<int>(y / resolution) + offset_static;

    // Ensure coordinates remain within valid bounds
    out.x = max(0, min(out_x, static_cast<int>(global_grid.size()) - 1));
    out.y = max(0, min(out_y, static_cast<int>(global_grid.size()) - 1));

    return out;
};




void GridManager::compute_step_heights_local(int window_radius_cells)
{
    if (local_grid.empty() || local_grid[0].empty())
    {
      std::cerr << "Error: Local grid not initialized." << std::endl;
      return;
    }

    const int H = static_cast<int>(local_grid.size());
    const int W = static_cast<int>(local_grid[0].size());

    // Clamp radius so it stays inside the grid
    int R = std::max(1, std::min(window_radius_cells, std::min(H, W) / 2));

    TerrainGrid copy = local_grid;  // to avoid using updated heights during computation

    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            const TerrainCell& center = copy[y][x];

            if (center.num_points == 0) {
                local_grid[y][x].height = 0.0;
                continue;
            }

            double center_z = center.mean_z;
            double max_dz = 0.0;

            // Neighborhood window
            for (int ny = y - R; ny <= y + R; ++ny)
            {
                if (ny < 0 || ny >= H) continue;
                for (int nx = x - R; nx <= x + R; ++nx)
                {
                    if (nx < 0 || nx >= W) continue;

                    const TerrainCell& nb = copy[ny][nx];
                    if (nb.num_points == 0) continue;

                    double dz = std::abs(nb.mean_z - center_z);
                    if (dz > max_dz)
                        max_dz = dz;
                }
            }

            local_grid[y][x].height = max_dz;  // step height in meters
        }
    }
};



// Get cell from grid
TerrainCell GridManager::get_cell(TerrainGrid& grid, int r, int c, int indx) const
{
    if (r < 0 || r >= static_cast<int>(grid.size()) || c < 0 || c >= static_cast<int>(grid[0].size()))
    {
        throw std::out_of_range("GridManager::get - Index out of range");
    }
    return grid[r][c];
}

