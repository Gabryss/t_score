/**
 * @file Ransac.hpp
 * @author XX.XX@uni.XX
 * @version 0.1
 * @date 2024-09-16
 * 
 * @copyright XX XX | 2024
 * @brief Implementation file for Class RansacAlgorithm.
 * @details This is a C++ wrapper with slight modification of a Ransac algorithm implementation.
 */
// #ifndef RANSAC_H
// #define RANSAC_H

#include <vector>
#include <iostream>
#include <random>  // uniform_int_distribution
#include <cmath>
#include <pcl/point_types.h>


using namespace std;

class RansacAlgorithm
{
    public:
        RansacAlgorithm() {};           
        ~RansacAlgorithm() {};

        // ===========================
        // Attributes
        // ===========================
        int t;                           // Threshold value to evaluate algorithm performance (inliners)
        int k=100;                       // Number of iterations allowed in the algorithm

        // vector<pcl::PointXYZI> data;     // Input data
        vector<double> distances;         // Distances from point to plane
        vector<double> temp_distances;    // Temp distance vector
        
        vector<double> bestFit;           // Model parameters


        // ===========================
        // Methods
        // ===========================
        void FitPlane(int t, vector<pcl::PointXYZ>& data, vector<double>& bestFit);
        void ResetState();

    protected:
        // ===========================
        // Methods
        // ===========================
        vector<double> PlaneEquation(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3);
        double CalculateDistanceFromPlane(const pcl::PointXYZ& points, const vector<double>& plane_eq);
        int CountInliers(vector<pcl::PointXYZ>& data, vector<double>& plane_eq, int t);
        
};
