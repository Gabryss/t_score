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
};


// Create a grid
void GridManager::create_grid(TerrainGrid grid, int size_m_x, int size_m_y, float res)
{
    grid.clear();
    num_cells_x = static_cast<int>(std::ceil(size_m_x / res));
    num_cells_y = static_cast<int>(std::ceil(size_m_y / res));

    grid.resize(num_cells_x);
    for(int ind_x=0; ind_x<num_cells_x; ind_x++)
    {
        grid[ind_x].resize(num_cells_y);
        for(int ind_y=0; ind_y<num_cells_y; ind_y++)
        {
            grid[ind_x][ind_y].resize(3);
            grid[ind_x][ind_y][0] = 0.0;
            grid[ind_x][ind_y][1] = 0.0;
            grid[ind_x][ind_y][2] = 0.0;
        }
    }
};



// Update global map
void GridManager::update_map(int offset_x, int offset_y)
{
  // Loop on each cell of the local grid
  for(int local_ind_x=0; local_ind_x<roughness.TGridLocal.size(); local_ind_x++)
  {
    for(int local_ind_y=0; local_ind_y<roughness.TGridLocal[0].size(); local_ind_y++)
    {


      coordinates_grid global_coord = coord_local_to_global({local_ind_x, local_ind_y});

      // Check if the computed global cell is within the bounds of the global grid.&
      if (global_coord[0] < 0 || global_coord[0] >= static_cast<int>(global_grid.size()) ||
          global_coord[1] < 0 || global_coord[1] >= static_cast<int>(global_grid.size()))
      {
        // Skip cells that fall outside the global grid.
        RCLCPP_ERROR(this->get_logger(), "Error: Local grid outside the global grid.");
        continue;
      } 

      if (roughness.TGridLocal[local_ind_x][local_ind_y][0]>0) // Only takes the computed values
      {
        for (int i=0; i<9; i++)
        {
          if ((i!=2) && (i!=3) && (i!=4) && (i!=5) && (i!=7))
          {
            global_grid[global_coord[0]][global_coord[1]][i] = roughness.TGridLocal[local_ind_x][local_ind_y][i];
          }
          else
          {
            coordinates_grid index_pose = coord_local_to_global({local_ind_x, local_ind_y});
            if((index_pose[0] == previous_cell[0]) && (index_pose[1] == previous_cell[1]))
            {
              global_grid[previous_cell[0]][previous_cell[1]][i] = roughness.TGridLocal[local_ind_x][local_ind_y][i];
            }
          }
        }
          

          //Save current cell
          // roughness.SaveCellASPC(local_ind_x, local_ind_y, vector_roughness_lidar_raw.size());
        // }
      }

      // global_grid[global_coord[0]][global_coord[1]] = roughness.TGridLocal[local_ind_x][local_ind_y];
          //Save current cell
          // roughness.SaveCellASPC(local_ind_x, local_ind_y, vector_roughness_lidar_raw.size());
    }
  }
};




// Get value from grid
double GridManager::get(TerrainGrid grid, int r, int c, int indx) const
{
    if (r < 0 || r >= static_cast<int>(grid.size()) || c < 0 || c >= static_cast<int>(grid[0].size()))
    {
        throw std::out_of_range("GridManager::get - Index out of range");
    }
    return grid[r][c][indx];
}

