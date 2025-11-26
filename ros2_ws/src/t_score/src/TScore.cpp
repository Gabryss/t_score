/**
 * @file TScore.cpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2025-11-19
 * 
 * @copyright Gabriel Garcia | 2025
 * @brief Implementation file for Class TScore (Traversability score).
 * @details This is a group of method to handle terrain traversability computation.
 */


#include "TScore.hpp"


// =====================================================
// Process Ransac on a pointcloud cell
// =====================================================
void TScore::FitPlane(int t, std::vector<pcl::PointXYZ>& data, std::vector<double>& bestFit)
{
    bestFit.clear();
    distances.clear();

    // Not enough points to define a plane
    if (data.size() < 3) {
        return;
    }

    int best_inliers = -1;  // allow 0 inliers to still be "best"
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist_data(0, data.size() - 1);

    for (int i = 0; i < k; ++i)
    {
        int indx_1 = dist_data(rng);
        int indx_2 = dist_data(rng);
        int indx_3 = dist_data(rng);

        while (indx_2 == indx_1) indx_2 = dist_data(rng);
        while (indx_3 == indx_1 || indx_3 == indx_2) indx_3 = dist_data(rng);

        std::vector<double> plane = PlaneEquation(data[indx_1], data[indx_2], data[indx_3]);


        // Reject planes where the normal is nearly horizontal (wall or noise)
        double A = plane[0];
        double B = plane[1];
        double C = plane[2];
        double norm = std::sqrt(A*A + B*B + C*C);
        if (norm < 1e-6) continue;

        A /= norm; 
        B /= norm;
        C /= norm;

        // Reject steep/non-ground normals
        if (std::abs(C) < 0.87) continue;   // <-- 60° max slope allowed

        int inlier_count = CountInliers(data, plane, t);  // fills temp_distances

        // >= so that even when all inlier_count==0, the last plane is still taken
        if (inlier_count >= best_inliers)
        {
            best_inliers = inlier_count;
            distances = temp_distances;   // copy current distances
            bestFit = plane;
        }
    }

    // Optional safety: if for some reason distances is still empty, fall back
    if (distances.empty())
    {
        // compute distances once for the last plane, or just set roughness=0
        for (const auto& p : data)
        {
            double d = CalculateDistanceFromPlane(p, bestFit);
            distances.push_back(d);
        }
    }
};



vector<double> TScore::PlaneEquation(const pcl::PointXYZ& p1,
                                     const pcl::PointXYZ& p2,
                                     const pcl::PointXYZ& p3)
{
    vector<double> v1({p2.x - p1.x, p2.y - p1.y, p2.z - p1.z});
    vector<double> v2({p3.x - p1.x, p3.y - p1.y, p3.z - p1.z});

    double A = v1[1]*v2[2] - v2[1]*v1[2];
    double B = v2[0]*v1[2] - v1[0]*v2[2];
    double C = v1[0]*v2[1] - v2[0]*v1[1];

    // Reject degenerate planes
    double norm = std::sqrt(A*A + B*B + C*C);
    if (norm < 1e-6)
        return {0,0,1,0}; // flat fallback

    // Normalize
    A /= norm;
    B /= norm;
    C /= norm;

    // Correct plane equation
    double D = -(A*p1.x + B*p1.y + C*p1.z);

    return {A,B,C,D};
};




// =====================================================
// Roughness
// =====================================================

double TScore::CalculateRoughness(std::vector<double>& distances)
{
    if (distances.empty()) {
        return 0.0;
    }

    double mean = std::accumulate(distances.begin(), distances.end(), 0.0) /
                  static_cast<double>(distances.size());

    double var = 0.0;
    for (double d : distances)
    {
        double diff = d - mean;
        var += diff * diff;
    }
    var /= static_cast<double>(distances.size());

    return std::sqrt(var);
};



// =====================================================
// Slope calculation
// =====================================================
double TScore::CalculateSlope(const std::vector<double>& plane_eq)
{
    // plane_eq = {A, B, C, D} from PlaneEquation
    if (plane_eq.size() < 3) {
        return 0.0;
    }

    double A = plane_eq[0];
    double B = plane_eq[1];
    double C = plane_eq[2];
    std::cerr << "Normal: " << A << " " << B << " " << C << "\n";


    // Normalize the normal vector (A,B,C)
    double norm = std::sqrt(A*A + B*B + C*C);
    if (norm < 1e-9) {
        // Degenerate plane → treat as flat
        return 0.0;
    }

    A /= norm;
    B /= norm;
    C /= norm;

    // Now (A,B,C) is unit length.
    // Vertical axis is (0,0,1).
    // cos(theta) = |dot(n, z_axis)| = |C|
    double cos_theta = std::clamp(std::abs(C), 0.0, 1.0);

    // theta = angle between plane normal and vertical.
    // This is actually the slope angle of the surface.
    double slope_rad = std::acos(cos_theta);   // in radians

    return slope_rad;
}


double TScore::CalculateMeanZ(const std::vector<pcl::PointXYZ>& data)
{
    if (data.empty()) return 0.0;

    double sum = 0.0;
    for (const auto& p : data) {
        sum += p.z;
    }
    return sum / static_cast<double>(data.size());
};


double TScore::calculateTScore(double slope, double roughness, double height, bool traversable)
{

    if (!traversable)
        return 100;   // non-traversable cell has T-Score = 100

    // Normalized metrics
    double S = std::clamp(slope     / s_crit, 0.0, 1.0);
    double R = std::clamp(roughness / r_crit, 0.0, 1.0);
    double H = std::clamp(height    / h_crit, 0.0, 1.0);

    // Combined scalar traversability
    double Risk = (w_s * S) + (w_r * R) + (w_h * H);
    double t_score = std::clamp(S, 0.0, 1.0);

    // double t_score = 1.0 - Risk;

    // std::cerr << "T-Score calculation: "
    //           << "slope=" << slope << " S=" << S << ", "
    //           << "roughness=" << roughness << " R=" << R << ", "
    //           << "height=" << height << " H=" << H << " => "
    //           << "Risk=" << Risk << ", T-Score=" << t_score << std::endl;

    // Map into [0,100]
    return std::clamp(t_score * 100.0, 0.0, 100.0);
};



// =====================================================
// TOOLS
// =====================================================

double TScore::CalculateDistanceFromPlane(const pcl::PointXYZ& point,
                                          const vector<double>& plane_eq)
{
    if (plane_eq.size() < 4) {
        return 0.0;
    }

    double A = plane_eq[0];
    double B = plane_eq[1];
    double C = plane_eq[2];
    double D = plane_eq[3];

    double down = std::sqrt(A*A + B*B + C*C);
    if (down < 1e-9) {
        // Invalid/degenerate plane → treat as zero distance
        return 0.0;
    }

    double up = std::abs(A * point.x + B * point.y + C * point.z + D);
    return up / down;
};



int TScore::CountInliers(vector<pcl::PointXYZ>& data, vector<double>& plane_eq, double t)
{
    int inliners_number = 0;
    temp_distances.clear();

    for(long unsigned int i=0; i<data.size(); i++)
    {
        double distance = CalculateDistanceFromPlane(data[i], plane_eq);

        if(distance < t)
        {
            temp_distances.push_back(distance);
            inliners_number++;
        }
    }
    return inliners_number;
};

void TScore::ResetState()
{
      *this = {};
};