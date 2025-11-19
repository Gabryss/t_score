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
#include <iostream>
#include <vector>
#include <stdexcept>


using namespace std;

// Each cell is a vector<double> of size 3 representing terrain state.
using TerrainCell = vector<double>;  
// The global grid is a 2D grid where each cell is a TerrainCell.
using TerrainGrid = vector<vector<TerrainCell>>;

class DynamicGlobalGrid 
{
    public:
        DynamicGlobalGrid(/* args */) {};
        ~DynamicGlobalGrid() {};
        // ===========================
        // Attributes
        // ===========================
        TerrainGrid grid;
        float resolution; // Resolution of the cells (cm)
        int originRow = 0;  // The row in 'grid' that corresponds to world row 0.
        int originCol = 0;  // The column in 'grid' that corresponds to world col 0.
        // ===========================
        // Methods
        // ===========================
        int get(int r, int c, int indx) const;
        void placeLocalGrid(const TerrainGrid &localGrid, int rowOffset, int colOffset);
        void print() const;



    protected:
        // ===========================
        // Attributes
        // ===========================
        


        // ===========================
        // Methods
        // ===========================
        pair<size_t, size_t> worldToIndex(int worldRow, int worldCol) const;
        void ensureSizeForWorldCoordinates(int worldTop, int worldLeft, int worldBottom, int worldRight);
};
