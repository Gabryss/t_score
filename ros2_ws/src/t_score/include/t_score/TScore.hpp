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
#include <pcl/point_types.h>

using namespace std;


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
        void FitPlane(int t, vector<pcl::PointXYZ>& data, vector<double>& bestFit);
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



