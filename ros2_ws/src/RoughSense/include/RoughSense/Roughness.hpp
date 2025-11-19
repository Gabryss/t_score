/**
 * @file Roughness.hpp
 * @author XX.XX@uni.XX
 * @version 0.1
 * @date 2024-09-20
 * 
 * @copyright XX XX | 2024
 * @brief Implementation file for Class Roughness.
 * @details This is a C++ Roughness library taking a PCL point cloud as an input and returning an image. CalculateRoughness is the core method of this library.
 */
#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>


// PCL libraries
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// OpenCV library
#include <opencv2/opencv.hpp>

// Custom libraries
#include "Ransac.hpp"
#include "MapCreator.hpp"
#include "DynamicGrid.hpp"



using namespace cv;

using namespace std;

// Each cell is a vector<double> of size 3 representing terrain state.
using TerrainCell = vector<double>;  
// The global grid is a 2D grid where each cell is a TerrainCell.
using TerrainGrid = vector<vector<TerrainCell>>;


class Roughness
{
    public:
        Roughness(/* args */) {};
        ~Roughness() {};

        // ===========================
        // Attributes
        // ===========================
        // Traversability grids
        TerrainGrid TGridLocal; //Grid of std_deviation, slope and gradient for each cell - Local
        
        // Point cloud grids
        vector<vector<vector<pcl::PointXYZ>>> PCGrid; //Grid with cells filled with points from the PC
        vector<vector<vector<pcl::PointXYZ>>> PCLowGrid; //Grid with cells filled with points from the PC
        

        vector<float> imu_data;
        vector<double> pose = {0.0,0.0,0.0};
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        float resolution;
        int low_grid_resolution=4;
        float local_size;
        float global_size;
        int nb_cells_global;

        float roughness_shift=0;
        float height=1;
        float min_height=0;
        float min_roughness=0;
        bool stylized=false;
        int ransac_iterations;
        unsigned int nb_cells;
        float roughness_lidar_threshold=1;
        float roughness_imu_threshold=1;

        // Traversability image conversion
        MapCreator map_creator;
        Mat image_local_roughness;
        Mat image_global_roughness;



        // ===========================
        // Methods
        // ===========================
        double CalculateStd(vector<double>& distances);
        double roughness_normalization(double value, float threshold);
        double calculateNorm(double linear_accel_x, double linear_accel_y, double linear_accel_z);

        //Main methods
        void CalculatePCRoughness(pcl::PointCloud<pcl::PointXYZ>::Ptr Data_in);
        void CalculateObservedRoughness(vector<double> observed_values, vector<int> previous_cell_coordinates);

        void DisplayGrid(); //For debug purposes
        void saveEntireGridToPCD(const vector<vector<vector<pcl::PointXYZ>>>& PC_Grid);

        
    protected:
        // ===========================
        // Attributes
        // ===========================
        pcl::PointCloud<pcl::PointXYZ>::Ptr Data_in;
        float LETHAL = 100.0;

        // ===========================
        // Methods
        // ===========================
        void ImportPCD(string path);
        void CreateTGridLocal();
        void CreatePCGrid();
        void FillPCGrid();
        void FillTGridLocal();
        void saveCellToPCD(const vector<vector<vector<pcl::PointXYZ>>>& PCGrid, size_t indx, size_t indy);


};
