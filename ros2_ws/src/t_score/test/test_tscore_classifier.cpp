#include "t_score/TScore.hpp"

#include <iostream>
#include <vector>

namespace
{
void addFloor(std::vector<pcl::PointXYZ>& points, double z_offset = 0.0)
{
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 4; ++y)
            points.emplace_back(0.05f * x, 0.05f * y, static_cast<float>(z_offset + 0.01 * x));
}

void addRoof(std::vector<pcl::PointXYZ>& points)
{
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 4; ++y)
            points.emplace_back(0.05f * x, 0.05f * y, static_cast<float>(2.0 + 0.01 * x));
}

TerrainAnalysisConfig testConfig()
{
    TerrainAnalysisConfig cfg;
    cfg.enable_ground_layer_filter = true;
    cfg.enable_column_classifier = true;
    cfg.min_points = 6;
    cfg.min_floor_points = 8;
    cfg.obstacle_min_points = 12;
    cfg.ceiling_min_points = 12;
    cfg.ground_band_above = 0.25;
    cfg.ceiling_ignore_height = 0.60;
    cfg.obstacle_min_height = 0.35;
    cfg.wall_min_vertical_span = 0.80;
    cfg.wall_min_points = 12;
    cfg.wall_cells_as_obstacles = true;
    cfg.require_floor_for_obstacle = true;
    return cfg;
}

bool expect(bool condition, const std::string& message)
{
    if (!condition)
        std::cerr << "FAILED: " << message << '\n';
    return condition;
}
}

int main()
{
    TScore scorer;
    TerrainAnalysisConfig cfg = testConfig();

    std::vector<pcl::PointXYZ> floor;
    addFloor(floor);
    TerrainAnalysis floor_analysis = scorer.AnalyzeCell(floor, cfg);
    if (!expect(floor_analysis.known, "floor should be known") ||
        !expect(floor_analysis.traversable, "floor should be traversable") ||
        !expect(!floor_analysis.obstacle, "floor should not be obstacle") ||
        !expect(floor_analysis.layer_class == CellLayerClass::Floor, "floor class"))
        return 1;

    std::vector<pcl::PointXYZ> floor_with_roof;
    addFloor(floor_with_roof);
    addRoof(floor_with_roof);
    TerrainAnalysis roof_analysis = scorer.AnalyzeCell(floor_with_roof, cfg);
    if (!expect(roof_analysis.known, "floor+roof should be known") ||
        !expect(roof_analysis.traversable, "floor+roof should remain traversable") ||
        !expect(!roof_analysis.obstacle, "roof should not create obstacle") ||
        !expect(roof_analysis.layer_class == CellLayerClass::Floor, "floor+roof class"))
        return 1;

    std::vector<pcl::PointXYZ> floor_with_obstacle;
    addFloor(floor_with_obstacle);
    for (int i = 0; i < 12; ++i)
        floor_with_obstacle.emplace_back(0.02f * i, 0.01f * i, static_cast<float>(0.45 + 0.005 * i));
    TerrainAnalysis obstacle_analysis = scorer.AnalyzeCell(floor_with_obstacle, cfg);
    if (!expect(obstacle_analysis.known, "floor+obstacle should be known") ||
        !expect(obstacle_analysis.obstacle, "floor+obstacle should be obstacle") ||
        !expect(!obstacle_analysis.traversable, "floor+obstacle should not be traversable") ||
        !expect(obstacle_analysis.cost >= 100.0, "floor+obstacle should be lethal") ||
        !expect(obstacle_analysis.layer_class == CellLayerClass::FloorWithObstacle, "floor+obstacle class"))
        return 1;

    std::vector<pcl::PointXYZ> wall_only;
    for (int z = 0; z < 20; ++z)
        wall_only.emplace_back(0.0f, 0.02f * (z % 3), 0.05f * z);
    TerrainAnalysis wall_analysis = scorer.AnalyzeCell(wall_only, cfg);
    if (!expect(wall_analysis.known, "wall should be known when walls are obstacles") ||
        !expect(wall_analysis.obstacle, "wall should be obstacle") ||
        !expect(!wall_analysis.traversable, "wall should not be traversable") ||
        !expect(wall_analysis.layer_class == CellLayerClass::WallOrVerticalSurface, "wall class"))
        return 1;

    std::cout << "test_tscore_classifier passed\n";
    return 0;
}
