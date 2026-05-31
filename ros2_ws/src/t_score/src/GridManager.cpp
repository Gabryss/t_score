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
    global_origin_x = -static_cast<double>(global_grid[0].size()) * resolution / 2.0;
    global_origin_y = -static_cast<double>(global_grid.size()) * resolution / 2.0;
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

GridCoord GridManager::pose_to_grid_coordinates(double x, double y)
{
    return pose_to_grid_coordinates(global_grid, x, y);
};

GridCoord GridManager::pose_to_grid_coordinates(const TerrainGrid& grid, double x, double y) const
{
    GridCoord out;

    if (grid.empty() || grid[0].empty())
    {
      std::cerr << "Error: Grid not initialized." << std::endl;
      return out;
    }

    double origin_x = global_origin_x;
    double origin_y = global_origin_y;

    if (&grid != &global_grid)
    {
        origin_x = -static_cast<double>(grid[0].size()) * resolution / 2.0;
        origin_y = -static_cast<double>(grid.size()) * resolution / 2.0;
    }

    out.x = static_cast<int>(std::floor((x - origin_x) / resolution));
    out.y = static_cast<int>(std::floor((y - origin_y) / resolution));

    return out;
};

bool GridManager::is_inside_global_grid(double x, double y, double margin_m) const
{
    if (global_grid.empty() || global_grid[0].empty())
        return false;

    const double max_x = global_origin_x + static_cast<double>(global_grid[0].size()) * resolution;
    const double max_y = global_origin_y + static_cast<double>(global_grid.size()) * resolution;

    return x >= global_origin_x + margin_m &&
           y >= global_origin_y + margin_m &&
           x <= max_x - margin_m &&
           y <= max_y - margin_m;
}

GridExpansion GridManager::expand_global_grid_to_include(double x,
                                                         double y,
                                                         double margin_m,
                                                         double growth_step_m,
                                                         double max_size_m)
{
    GridExpansion result;
    if (global_grid.empty() || global_grid[0].empty() || resolution <= 0.0f)
        return result;

    if (is_inside_global_grid(x, y, margin_m))
        return result;

    const int old_h = static_cast<int>(global_grid.size());
    const int old_w = static_cast<int>(global_grid[0].size());
    const int max_cells = std::max(old_w, static_cast<int>(std::ceil(max_size_m / resolution)));
    const int step_cells = std::max(1, static_cast<int>(std::ceil(growth_step_m / resolution)));

    int add_left = 0;
    int add_right = 0;
    int add_bottom = 0;
    int add_top = 0;

    double min_x = global_origin_x;
    double min_y = global_origin_y;
    double max_x = global_origin_x + static_cast<double>(old_w) * resolution;
    double max_y = global_origin_y + static_cast<double>(old_h) * resolution;

    while (x < min_x + margin_m && old_w + add_left + add_right < max_cells)
    {
        const int add = std::min(step_cells, max_cells - (old_w + add_left + add_right));
        add_left += add;
        min_x -= static_cast<double>(add) * resolution;
    }

    while (x > max_x - margin_m && old_w + add_left + add_right < max_cells)
    {
        const int add = std::min(step_cells, max_cells - (old_w + add_left + add_right));
        add_right += add;
        max_x += static_cast<double>(add) * resolution;
    }

    while (y < min_y + margin_m && old_h + add_bottom + add_top < max_cells)
    {
        const int add = std::min(step_cells, max_cells - (old_h + add_bottom + add_top));
        add_bottom += add;
        min_y -= static_cast<double>(add) * resolution;
    }

    while (y > max_y - margin_m && old_h + add_bottom + add_top < max_cells)
    {
        const int add = std::min(step_cells, max_cells - (old_h + add_bottom + add_top));
        add_top += add;
        max_y += static_cast<double>(add) * resolution;
    }

    if (add_left == 0 && add_right == 0 && add_bottom == 0 && add_top == 0)
        return result;

    TerrainGrid expanded;
    expanded.resize(old_h + add_bottom + add_top);
    for (auto& row : expanded)
        row.resize(old_w + add_left + add_right);

    for (int y_idx = 0; y_idx < old_h; ++y_idx)
    {
        for (int x_idx = 0; x_idx < old_w; ++x_idx)
        {
            expanded[y_idx + add_bottom][x_idx + add_left] = std::move(global_grid[y_idx][x_idx]);
        }
    }

    global_grid = std::move(expanded);
    global_origin_x -= static_cast<double>(add_left) * resolution;
    global_origin_y -= static_cast<double>(add_bottom) * resolution;

    result.expanded = true;
    result.shift_x = add_left;
    result.shift_y = add_bottom;
    return result;
}

void GridManager::clear_grid(TerrainGrid& grid)
{
    for (auto& row : grid)
    {
        for (auto& cell : row)
        {
            TerrainCell empty;
            cell = std::move(empty);
        }
    }
}




void GridManager::compute_step_heights(TerrainGrid& grid, int window_radius_cells)
{
    if (grid.empty() || grid[0].empty())
    {
      std::cerr << "Error: Grid not initialized." << std::endl;
      return;
    }

    const int H = static_cast<int>(grid.size());
    const int W = static_cast<int>(grid[0].size());

    // Clamp radius so it stays inside the grid
    int R = std::max(1, std::min(window_radius_cells, std::min(H, W) / 2));

    TerrainGrid copy = grid;  // to avoid using updated heights during computation

    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            const TerrainCell& center = copy[y][x];

            if (!center.known) {
                grid[y][x].height = 0.0;
                continue;
            }

            double center_z = center.z_p50;
            double max_dz = center.height;

            // Neighborhood window
            for (int ny = y - R; ny <= y + R; ++ny)
            {
                if (ny < 0 || ny >= H) continue;
                for (int nx = x - R; nx <= x + R; ++nx)
                {
                    if (nx < 0 || nx >= W) continue;

                    const TerrainCell& nb = copy[ny][nx];
                    if (!nb.known) continue;

                    double dz = std::abs(nb.z_p50 - center_z);
                    if (dz > max_dz)
                        max_dz = dz;
                }
            }

            grid[y][x].height = max_dz;  // step height in meters
        }
    }
};

void GridManager::compute_step_heights_local(int window_radius_cells)
{
    compute_step_heights(local_grid, window_radius_cells);
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
