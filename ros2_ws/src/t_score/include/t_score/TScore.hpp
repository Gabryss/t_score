/**
 * @file TScore.hpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2025-11-19
 * 
 * @copyright Gabriel Garcia | 2025
 * @brief Implementation file for Class Tscore (Traversability score).
 * @details This is a group of method to handle terrain traversability computation.
 */


#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <random>  // uniform_int_distribution
#include <algorithm>
#include <numeric>
#include <pcl/point_types.h>
#include <Eigen/Dense>

using namespace std;

enum class CellLayerClass
{
    Unknown,
    Floor,
    FloorWithObstacle,
    WallOrVerticalSurface,
    CeilingOnly
};

struct TerrainAnalysisConfig
{
    int min_points = 6;
    int confidence_full_points = 30;
    double slope_critical = 0.52;
    double roughness_critical = 0.10;
    double height_critical = 0.25;
    double max_traversable_slope = 0.70;
    double max_traversable_roughness = 0.20;
    double max_traversable_height = 0.25;
    double slope_weight = 0.4;
    double roughness_weight = 0.3;
    double height_weight = 0.3;
    double confidence_weight = 0.15;
    bool enable_ground_layer_filter = true;
    double ground_quantile = 0.08;
    double ground_band_below = 0.05;
    double ground_band_above = 0.25;
    double ceiling_ignore_height = 1.20;
    double obstacle_min_height = 0.25;
    int obstacle_min_points = 6;
    bool enable_column_classifier = true;
    int min_floor_points = 8;
    int ceiling_min_points = 12;
    double wall_min_vertical_span = 0.80;
    int wall_min_points = 12;
    bool wall_cells_as_obstacles = true;
    bool require_floor_for_obstacle = true;
};

struct TerrainAnalysis
{
    bool known = false;
    bool traversable = false;
    bool obstacle = false;
    CellLayerClass layer_class = CellLayerClass::Unknown;
    int num_points = 0;
    int raw_num_points = 0;
    int obstacle_points = 0;
    int floor_points = 0;
    int ceiling_points = 0;
    double ground_z = 0.0;
    double slope = 0.0;
    double roughness = 0.0;
    double height = 0.0;
    double mean_z = 0.0;
    double z_min = 0.0;
    double z_max = 0.0;
    double z_p05 = 0.0;
    double z_p50 = 0.0;
    double z_p95 = 0.0;
    double confidence = 0.0;
    double cost = -1.0;
};

struct CellLayerClassification
{
    CellLayerClass layer_class = CellLayerClass::Unknown;
    std::vector<pcl::PointXYZ> floor_points;
    int obstacle_points = 0;
    int ceiling_points = 0;
    double ground_z = 0.0;
    double z_min = 0.0;
    double z_max = 0.0;
    double z_p05 = 0.0;
    double z_p50 = 0.0;
    double z_p95 = 0.0;
};


class TScore 
{
    public:
        TScore(/* args */) {};
        ~TScore() {};
        // ===========================
        // Attributes
        // ===========================
        double t=0.1;                           // Threshold value to evaluate algorithm performance (inliners)
        int k=100;                       // Number of iterations allowed in the algorithm

        // vector<pcl::PointXYZI> data;     // Input data
        vector<double> distances;         // Distances from point to plane
        vector<double> temp_distances;    // Temp distance vector
        
        vector<double> bestFit;           // Model parameters

        // Critical values
        double s_crit = 0.52;  // rad ≈ 30°
        double r_crit = 0.3;  // m
        double h_crit = 10.10;  // m

        // Weights
        double w_s = 0.4;  // slope most important
        double w_r = 0.3;
        double w_h = 0.3;

        // ===========================
        // Methods
        // ===========================
        TerrainAnalysis AnalyzeCell(const vector<pcl::PointXYZ>& data, const TerrainAnalysisConfig& config) const;
        CellLayerClassification ClassifyCellLayers(const vector<pcl::PointXYZ>& data, const TerrainAnalysisConfig& config) const;
        double calculateTScore(double slope, double roughness, double height, double confidence, bool traversable, const TerrainAnalysisConfig& config) const;
        void FitPlane(double t, vector<pcl::PointXYZ>& data, vector<double>& bestFit);
        void ResetState();
        double CalculateRoughness(vector<double>& distances);
        double CalculateSlope(const std::vector<double>& plane_eq);
        double CalculateMeanZ(const std::vector<pcl::PointXYZ>& data);
        double calculateTScore(double slope, double roughness, double height, bool traversable);

    protected:
        // ===========================
        // Methods
        // ===========================
        vector<double> PlaneEquation(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3);
        double CalculateDistanceFromPlane(const pcl::PointXYZ& points, const vector<double>& plane_eq);
        int CountInliers(vector<pcl::PointXYZ>& data, vector<double>& plane_eq, double t);
        
};
