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

namespace
{
double percentile(std::vector<double> values, double q)
{
    if (values.empty())
        return 0.0;

    std::sort(values.begin(), values.end());
    const double pos = std::clamp(q, 0.0, 1.0) * static_cast<double>(values.size() - 1);
    const auto lo = static_cast<size_t>(std::floor(pos));
    const auto hi = static_cast<size_t>(std::ceil(pos));
    if (lo == hi)
        return values[lo];
    const double t = pos - static_cast<double>(lo);
    return values[lo] * (1.0 - t) + values[hi] * t;
}
}

CellLayerClassification TScore::ClassifyCellLayers(const std::vector<pcl::PointXYZ>& data,
                                                   const TerrainAnalysisConfig& config) const
{
    CellLayerClassification out;
    if (data.empty())
        return out;

    std::vector<double> z_values;
    z_values.reserve(data.size());
    for (const auto& point : data)
        z_values.push_back(point.z);

    out.z_min = *std::min_element(z_values.begin(), z_values.end());
    out.z_max = *std::max_element(z_values.begin(), z_values.end());
    out.z_p05 = percentile(z_values, 0.05);
    out.z_p50 = percentile(z_values, 0.50);
    out.z_p95 = percentile(z_values, 0.95);
    out.ground_z = percentile(z_values, config.ground_quantile);

    const double floor_min_z = out.ground_z - std::max(0.0, config.ground_band_below);
    const double floor_max_z = out.ground_z + std::max(0.0, config.ground_band_above);
    const double obstacle_min_z = out.ground_z + std::max(0.0, config.obstacle_min_height);
    const double clearance_z = out.ground_z + std::max(config.obstacle_min_height, config.ceiling_ignore_height);

    out.floor_points.reserve(data.size());
    for (const auto& point : data)
    {
        if (point.z >= floor_min_z && point.z <= floor_max_z)
            out.floor_points.push_back(point);
        if (point.z >= obstacle_min_z && point.z <= clearance_z)
            ++out.obstacle_points;
        if (point.z > clearance_z)
            ++out.ceiling_points;
    }

    const bool has_floor = static_cast<int>(out.floor_points.size()) >= config.min_floor_points;
    const bool has_obstacle = out.obstacle_points >= config.obstacle_min_points;
    const bool has_ceiling = out.ceiling_points >= config.ceiling_min_points;
    const bool vertical_wall_candidate =
        (out.z_p95 - out.z_p05) >= config.wall_min_vertical_span &&
        static_cast<int>(data.size()) >= config.wall_min_points;

    if (has_floor)
    {
        out.layer_class = has_obstacle ? CellLayerClass::FloorWithObstacle : CellLayerClass::Floor;
        return out;
    }

    if (vertical_wall_candidate)
    {
        out.layer_class = CellLayerClass::WallOrVerticalSurface;
        return out;
    }

    if (has_ceiling)
    {
        out.layer_class = CellLayerClass::CeilingOnly;
        return out;
    }

    if (!config.require_floor_for_obstacle && has_obstacle)
    {
        out.layer_class = CellLayerClass::FloorWithObstacle;
        return out;
    }

    out.layer_class = CellLayerClass::Unknown;
    return out;
}

TerrainAnalysis TScore::AnalyzeCell(const std::vector<pcl::PointXYZ>& data, const TerrainAnalysisConfig& config) const
{
    TerrainAnalysis out;
    out.raw_num_points = static_cast<int>(data.size());
    out.num_points = out.raw_num_points;

    if (out.raw_num_points < config.min_points)
        return out;

    CellLayerClassification classification;
    const std::vector<pcl::PointXYZ>* analysis_points = &data;

    if (config.enable_ground_layer_filter || config.enable_column_classifier)
    {
        classification = ClassifyCellLayers(data, config);
        out.layer_class = classification.layer_class;
        out.ground_z = classification.ground_z;
        out.floor_points = static_cast<int>(classification.floor_points.size());
        out.obstacle_points = classification.obstacle_points;
        out.ceiling_points = classification.ceiling_points;
        out.z_min = classification.z_min;
        out.z_max = classification.z_max;
        out.z_p05 = classification.z_p05;
        out.z_p50 = classification.z_p50;
        out.z_p95 = classification.z_p95;

        if (classification.layer_class == CellLayerClass::Unknown ||
            classification.layer_class == CellLayerClass::CeilingOnly)
        {
            return out;
        }

        if (classification.layer_class == CellLayerClass::WallOrVerticalSurface)
        {
            if (!config.wall_cells_as_obstacles)
                return out;

            out.known = true;
            out.obstacle = true;
            out.traversable = false;
            out.num_points = out.raw_num_points;
            out.confidence = 1.0;
            out.mean_z = out.ground_z;
            out.height = std::max(0.0, out.z_p95 - out.z_p05);
            out.cost = 100.0;
            return out;
        }

        if (classification.layer_class == CellLayerClass::FloorWithObstacle)
        {
            out.known = true;
            out.obstacle = true;
            out.traversable = false;
            out.num_points = static_cast<int>(std::max(classification.floor_points.size(),
                                                       static_cast<size_t>(out.obstacle_points)));
            out.confidence = 1.0;
            out.mean_z = out.ground_z;
            out.height = std::max(0.0, std::min(out.z_max - out.ground_z, config.ceiling_ignore_height));
            out.cost = 100.0;
            return out;
        }

        if (static_cast<int>(classification.floor_points.size()) < config.min_points)
            return out;

        analysis_points = &classification.floor_points;
        out.num_points = static_cast<int>(classification.floor_points.size());
    }

    out.known = true;
    out.confidence = std::clamp(
        static_cast<double>(out.num_points - config.min_points) /
        static_cast<double>(std::max(1, config.confidence_full_points - config.min_points)),
        0.0,
        1.0);

    std::vector<double> z_values;
    z_values.reserve(analysis_points->size());
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& point : *analysis_points)
    {
        mean += Eigen::Vector3d(point.x, point.y, point.z);
        z_values.push_back(point.z);
    }
    mean /= static_cast<double>(analysis_points->size());

    out.mean_z = mean.z();
    out.z_min = *std::min_element(z_values.begin(), z_values.end());
    out.z_max = *std::max_element(z_values.begin(), z_values.end());
    out.z_p05 = percentile(z_values, 0.05);
    out.z_p50 = percentile(z_values, 0.50);
    out.z_p95 = percentile(z_values, 0.95);
    out.height = std::max(0.0, out.z_p95 - out.z_p05);

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& point : *analysis_points)
    {
        Eigen::Vector3d centered(point.x - mean.x(), point.y - mean.y(), point.z - mean.z());
        covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(analysis_points->size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success)
    {
        out.cost = -1.0;
        return out;
    }

    Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
    if (normal.z() < 0.0)
        normal = -normal;

    out.slope = std::acos(std::clamp(std::abs(normal.z()), 0.0, 1.0));

    double residual_sum = 0.0;
    double residual_sq_sum = 0.0;
    for (const auto& point : *analysis_points)
    {
        Eigen::Vector3d p(point.x, point.y, point.z);
        const double residual = std::abs(normal.dot(p - mean));
        residual_sum += residual;
        residual_sq_sum += residual * residual;
    }
    const double residual_mean = residual_sum / static_cast<double>(analysis_points->size());
    const double residual_var = std::max(0.0, residual_sq_sum / static_cast<double>(analysis_points->size()) - residual_mean * residual_mean);
    out.roughness = std::sqrt(residual_var);

    out.traversable =
        out.slope <= config.max_traversable_slope &&
        out.roughness <= config.max_traversable_roughness &&
        out.height <= config.max_traversable_height;

    out.cost = calculateTScore(out.slope, out.roughness, out.height, out.confidence, out.traversable, config);
    return out;
}

double TScore::calculateTScore(double slope,
                               double roughness,
                               double height,
                               double confidence,
                               bool traversable,
                               const TerrainAnalysisConfig& config) const
{
    if (!traversable)
        return 100.0;

    const double S = std::clamp(slope / config.slope_critical, 0.0, 1.0);
    const double R = std::clamp(roughness / config.roughness_critical, 0.0, 1.0);
    const double H = std::clamp(height / config.height_critical, 0.0, 1.0);
    const double C = 1.0 - std::clamp(confidence, 0.0, 1.0);

    const double risk =
        (config.slope_weight * S) +
        (config.roughness_weight * R) +
        (config.height_weight * H) +
        (config.confidence_weight * C);

    return std::clamp(risk * 100.0, 0.0, 100.0);
}


// =====================================================
// Process Ransac on a pointcloud cell
// =====================================================
void TScore::FitPlane(double t, std::vector<pcl::PointXYZ>& data, std::vector<double>& bestFit)
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
    double risk = (w_s * S) + (w_r * R) + (w_h * H);
    double t_score = std::clamp(risk, 0.0, 1.0);

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
