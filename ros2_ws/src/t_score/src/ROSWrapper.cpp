/**
 * @file ROSWrapper.cpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2025-11-19
 * 
 * @copyright Gabriel Garcia | 2025
 * @brief Implementation file for Class ROSWrapper.
 * @details This is a ROS wrapper that subscribe to a pointcloud and publish a traversability costmap.
 */
#include "ROSWrapper.hpp"

namespace
{
double getDouble(const rapidjson::Document& doc, const char* key, double fallback)
{
    return doc.HasMember(key) && doc[key].IsNumber() ? doc[key].GetDouble() : fallback;
}

int getInt(const rapidjson::Document& doc, const char* key, int fallback)
{
    return doc.HasMember(key) && doc[key].IsInt() ? doc[key].GetInt() : fallback;
}

bool getBool(const rapidjson::Document& doc, const char* key, bool fallback)
{
    return doc.HasMember(key) && doc[key].IsBool() ? doc[key].GetBool() : fallback;
}

std::string getString(const rapidjson::Document& doc, const char* key, const std::string& fallback)
{
    return doc.HasMember(key) && doc[key].IsString() ? doc[key].GetString() : fallback;
}

std::string normalizeNamespace(std::string ns)
{
    if (ns.empty() || ns == "/")
        return "";
    if (ns.front() != '/')
        ns.insert(ns.begin(), '/');
    while (ns.size() > 1 && ns.back() == '/')
        ns.pop_back();
    return ns;
}

std::string resolveTopicName(const std::string& topic, const std::string& ns)
{
    if (topic.empty() || ns.empty())
        return topic;
    if (topic.rfind(ns + "/", 0) == 0)
        return topic;
    if (topic.front() == '/')
        return ns + topic;
    return ns + "/" + topic;
}

int scaleToOccupancy(double value, double critical)
{
    if (critical <= 0.0)
        return 0;
    return static_cast<int>(std::round(std::clamp(value / critical, 0.0, 1.0) * 100.0));
}

struct TraversabilityStats
{
    int occupied = 0;
    int known = 0;
    int unknown = 0;
    int floor = 0;
    int floor_with_obstacle = 0;
    int wall = 0;
    int ceiling = 0;
    int floating_suppressed = 0;
    int lethal_obstacle = 0;
    int lethal_slope = 0;
    int lethal_roughness = 0;
    int lethal_height = 0;
    int graded = 0;
};
}



ROSWrapper::ROSWrapper(): Node("t_score_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    // =====================================================
    // GET CONFIG PARAMETERS
    // =====================================================
    this->declare_parameter("param_path", "");
    std::string path_parameters = this->get_parameter("param_path").as_string();
    get_parameters(path_parameters);

    resolution = getDouble(p, "map_resolution", 0.5);
    local_map_size = getDouble(p, "local_map_size", 5.0);
    global_map_size = getDouble(p, "global_map_size", 1000.0);

    update_frequency = getDouble(p, "update_frequency", 1.0);
    int update_period_ms = static_cast<int>(1000.0 / std::max(0.1f, update_frequency));
    map_frame_id = getString(p, "traversability_frame_id", "map");
    robot_frame_id = getString(p, "robot_frame_id", "base_footprint");
    const std::string ros_namespace = normalizeNamespace(getString(p, "ros_namespace", ""));
    debug_logging = getBool(p, "debug_logging", false);
    publish_debug_maps = getBool(p, "publish_debug_maps", true);
    rebuild_global_map_on_cloud = getBool(p, "rebuild_global_map_on_cloud", true);
    compute_on_cloud_update = getBool(p, "compute_on_cloud_update", false);
    publish_on_timer = getBool(p, "publish_on_timer", true);
    one_shot = getBool(p, "one_shot", false);
    allow_identity_pose_fallback = getBool(p, "allow_identity_pose_fallback", true);
    publish_global_on_update_only = getBool(p, "publish_global_on_update_only", true);
    publish_local_map = getBool(p, "publish_local_map", true);
    enable_footprint_inflation = getBool(p, "enable_footprint_inflation", enable_footprint_inflation);
    step_window_radius_cells = getInt(p, "step_window_radius_cells", 1);
    max_points_per_cell = std::max(3, getInt(p, "max_points_per_cell", 80));
    cloud_point_stride = std::max(1, getInt(p, "cloud_point_stride", 1));
    robot_radius = getDouble(p, "robot_radius", 0.45);
    global_map_growth_margin = getDouble(p, "global_map_growth_margin", 5.0);
    global_map_growth_step = getDouble(p, "global_map_growth_step", 20.0);
    global_map_max_size = getDouble(p, "global_map_max_size", std::max<double>(global_map_size, 300.0));
    floating_floor_neighbor_radius = getInt(p, "floating_floor_neighbor_radius", floating_floor_neighbor_radius);
    max_floor_height_jump = getDouble(p, "max_floor_height_jump", max_floor_height_jump);
    footprint_radius_cells = enable_footprint_inflation
        ? std::max(1, static_cast<int>(std::ceil(robot_radius / resolution)))
        : 0;

    analysis_config.min_points = getInt(p, "min_points_per_cell", analysis_config.min_points);
    analysis_config.confidence_full_points = getInt(p, "confidence_full_points", analysis_config.confidence_full_points);
    analysis_config.slope_critical = getDouble(p, "slope_critical", analysis_config.slope_critical);
    analysis_config.roughness_critical = getDouble(p, "roughness_critical", analysis_config.roughness_critical);
    analysis_config.height_critical = getDouble(p, "height_critical", analysis_config.height_critical);
    analysis_config.max_traversable_slope = getDouble(p, "max_traversable_slope", analysis_config.max_traversable_slope);
    analysis_config.max_traversable_roughness = getDouble(p, "max_traversable_roughness", analysis_config.max_traversable_roughness);
    analysis_config.max_traversable_height = getDouble(p, "max_traversable_height", analysis_config.max_traversable_height);
    analysis_config.slope_weight = getDouble(p, "slope_weight", analysis_config.slope_weight);
    analysis_config.roughness_weight = getDouble(p, "roughness_weight", analysis_config.roughness_weight);
    analysis_config.height_weight = getDouble(p, "height_weight", analysis_config.height_weight);
    analysis_config.confidence_weight = getDouble(p, "confidence_weight", analysis_config.confidence_weight);
    analysis_config.enable_ground_layer_filter = getBool(p, "enable_ground_layer_filter", analysis_config.enable_ground_layer_filter);
    analysis_config.ground_quantile = getDouble(p, "ground_quantile", analysis_config.ground_quantile);
    analysis_config.ground_band_below = getDouble(p, "ground_band_below", analysis_config.ground_band_below);
    analysis_config.ground_band_above = getDouble(p, "ground_band_above", analysis_config.ground_band_above);
    analysis_config.ceiling_ignore_height = getDouble(p, "ceiling_ignore_height", analysis_config.ceiling_ignore_height);
    analysis_config.obstacle_min_height = getDouble(p, "obstacle_min_height", analysis_config.obstacle_min_height);
    analysis_config.obstacle_min_points = getInt(p, "obstacle_min_points", analysis_config.obstacle_min_points);
    analysis_config.enable_column_classifier = getBool(p, "enable_column_classifier", analysis_config.enable_column_classifier);
    analysis_config.min_floor_points = getInt(p, "min_floor_points", analysis_config.min_floor_points);
    analysis_config.ceiling_min_points = getInt(p, "ceiling_min_points", analysis_config.ceiling_min_points);
    analysis_config.wall_min_vertical_span = getDouble(p, "wall_min_vertical_span", analysis_config.wall_min_vertical_span);
    analysis_config.wall_min_points = getInt(p, "wall_min_points", analysis_config.wall_min_points);
    analysis_config.wall_cells_as_obstacles = getBool(p, "wall_cells_as_obstacles", analysis_config.wall_cells_as_obstacles);
    analysis_config.require_floor_for_obstacle = getBool(p, "require_floor_for_obstacle", analysis_config.require_floor_for_obstacle);


    // Initialize grid
    grid_manager.create_local_global_grids(global_map_size, local_map_size, resolution);
    RCLCPP_INFO(this->get_logger(), "Global grid: %zu x %zu cells at %.3f m resolution",
                grid_manager.global_grid[0].size(), grid_manager.global_grid.size(), resolution);
    RCLCPP_INFO(this->get_logger(), "Local grid: %zu x %zu cells",
                grid_manager.local_grid[0].size(), grid_manager.local_grid.size());



    

    // =====================================================
    // TRANSFORM
    // =====================================================
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (publish_on_timer)
    {
        timer_tf_ = rclcpp::create_timer(
            this->get_node_base_interface(),
            this->get_node_timers_interface(),
            this->get_clock(),
            std::chrono::milliseconds(std::max(1, update_period_ms)),
            std::bind(&ROSWrapper::updateAndPublish, this)
        );
    }


    // =====================================================
    // POINT CLOUD
    // =====================================================
    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Subscribe to the point cloud topic    
    rclcpp::QoS pc_qos(getInt(p, "pc_qos_depth", 1));
    pc_qos.reliable();
    if (getBool(p, "pc_qos_transient_local", true))
        pc_qos.transient_local();
    else
        pc_qos.durability_volatile();

    sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
     resolveTopicName(getString(p, "pc_topic", "/rtabmap/cloud_map"), ros_namespace),
     pc_qos,
     std::bind(&ROSWrapper::pc_callback, this, _1));
    

    // Create publishers
    pub_t_score_local_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        resolveTopicName(getString(p, "traversability_topic_local", "/traversability_costmap_local"), ros_namespace), 10);
    pub_t_score_global_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        resolveTopicName(getString(p, "traversability_topic_global", "/traversability_costmap"), ros_namespace), 10);

    if (publish_debug_maps)
    {
        pub_slope_global_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            resolveTopicName("/t_score/slope_map", ros_namespace), 10);
        pub_roughness_global_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            resolveTopicName("/t_score/roughness_map", ros_namespace), 10);
        pub_height_global_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            resolveTopicName("/t_score/height_map", ros_namespace), 10);
        pub_confidence_global_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            resolveTopicName("/t_score/confidence_map", ros_namespace), 10);
    }

    RCLCPP_INFO(this->get_logger(), "Now: %.3f (sim time = %s)", 
    this->get_clock()->now().seconds(),
    this->get_parameter("use_sim_time").as_bool() ? "true" : "false");



};


ROSWrapper::~ROSWrapper() 
{
};


// =====================================================
// CALLBACKS
// =====================================================


void ROSWrapper::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!transformPointCloudToMap(msg, *cloud))
        return;

    if (rebuild_global_map_on_cloud)
    {
        for (const auto& coord : occupied_global_cells)
            grid_manager.global_grid[coord.y][coord.x] = TerrainCell{};
        occupied_global_cells.clear();
    }

    expandGlobalGridForCloud(*cloud);

    std::unordered_set<int64_t> occupied_ids;
    occupied_ids.reserve(cloud->points.size() / 8 + 1);

    // Update global map with new pointcloud data
    for (size_t i = 0; i < cloud->points.size(); i += static_cast<size_t>(cloud_point_stride))
    {
        const auto& point = cloud->points[i];
        auto [gx, gy] = grid_manager.pose_to_grid_coordinates(point.x, point.y);
        
        // Bounds check:
        if (gy < 0 || gy >= static_cast<int>(grid_manager.global_grid.size()))
            continue;
        if (gx < 0 || gx >= static_cast<int>(grid_manager.global_grid[0].size()))
            continue;
        
        TerrainCell& cell = grid_manager.global_grid[gy][gx];
        if (static_cast<int>(cell.points.size()) < max_points_per_cell)
        {
            cell.points.push_back(point);
            cell.num_points = static_cast<int>(cell.points.size());
        }

        const int64_t id = static_cast<int64_t>(gy) * static_cast<int64_t>(grid_manager.global_grid[0].size()) + gx;
        if (occupied_ids.insert(id).second)
            occupied_global_cells.push_back({gx, gy});
    }

    cloud_received = true;
    map_dirty = true;
    if (compute_on_cloud_update)
        updateAndPublish();
};



// =====================================================
// TRANSFORM
// =====================================================


void ROSWrapper::lookupTransform()
{
    updateRobotPose();
};

// =====================================================
// DATA PROCESSING
// =====================================================

void ROSWrapper::compute_t_score()
{
    analyze_global_grid();
    update_local_grid_from_global();
}




// =====================================================
// Publisher
// =====================================================

void ROSWrapper::publish_t_score_map(const TerrainGrid &grid, bool is_local)
{
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg = make_occupancy_grid_message(grid, is_local);

    if (is_local)
        pub_t_score_local_->publish(occupancy_grid_msg);
    else
        pub_t_score_global_->publish(occupancy_grid_msg);
}

void ROSWrapper::publish_debug_maps_for_grid(const TerrainGrid &grid, bool is_local)
{
    if (!publish_debug_maps || is_local)
        return;

    pub_slope_global_->publish(make_metric_grid_message(grid, false, "slope"));
    pub_roughness_global_->publish(make_metric_grid_message(grid, false, "roughness"));
    pub_height_global_->publish(make_metric_grid_message(grid, false, "height"));
    pub_confidence_global_->publish(make_metric_grid_message(grid, false, "confidence"));
}

bool ROSWrapper::updateRobotPose()
{
    try
    {
        transform_stamped = tf_buffer_->lookupTransform(map_frame_id, robot_frame_id, tf2::TimePointZero);
        robot_coordinates = {
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z
        };
        robot_coordinates_grid = grid_manager.pose_to_grid_coordinates(robot_coordinates[0], robot_coordinates[1]);
        has_robot_pose = true;

        if (debug_logging)
        {
            RCLCPP_DEBUG(this->get_logger(), "Robot coordinates: %.3f %.3f %.3f",
                         robot_coordinates[0], robot_coordinates[1], robot_coordinates[2]);
        }
        return true;
    }
    catch (const tf2::TransformException& ex)
    {
        if (allow_identity_pose_fallback && cloud_received)
        {
            robot_coordinates = {0.0, 0.0, 0.0};
            robot_coordinates_grid = grid_manager.pose_to_grid_coordinates(0.0, 0.0);
            has_robot_pose = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "Could not transform '%s' to '%s' (%s). Using identity pose fallback for map publication.",
                                 map_frame_id.c_str(), robot_frame_id.c_str(), ex.what());
            return true;
        }

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Could not transform '%s' to '%s': %s",
                             map_frame_id.c_str(), robot_frame_id.c_str(), ex.what());
        has_robot_pose = false;
        return false;
    }
}

bool ROSWrapper::transformPointCloudToMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                          pcl::PointCloud<pcl::PointXYZ>& output)
{
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(*msg, input);
    output.clear();
    output.reserve(input.size());

    const std::string source_frame = msg->header.frame_id.empty() ? map_frame_id : msg->header.frame_id;
    if (source_frame == map_frame_id)
    {
        output = std::move(input);
        return true;
    }

    geometry_msgs::msg::TransformStamped cloud_transform;
    try
    {
        rclcpp::Time stamp(msg->header.stamp);
        cloud_transform = tf_buffer_->lookupTransform(map_frame_id, source_frame, stamp);
    }
    catch (const tf2::TransformException&)
    {
        try
        {
            cloud_transform = tf_buffer_->lookupTransform(map_frame_id, source_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Could not transform cloud from '%s' to '%s': %s",
                                 source_frame.c_str(), map_frame_id.c_str(), ex.what());
            return false;
        }
    }

    for (const auto& point : input.points)
        output.push_back(transformPoint(point, cloud_transform));

    output.width = static_cast<uint32_t>(output.size());
    output.height = 1;
    output.is_dense = input.is_dense;
    return true;
}

pcl::PointXYZ ROSWrapper::transformPoint(const pcl::PointXYZ& point,
                                         const geometry_msgs::msg::TransformStamped& transform) const
{
    const auto& q_msg = transform.transform.rotation;
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    tf2::Matrix3x3 rotation(q);
    tf2::Vector3 input(point.x, point.y, point.z);
    tf2::Vector3 translation(transform.transform.translation.x,
                             transform.transform.translation.y,
                             transform.transform.translation.z);
    tf2::Vector3 output = rotation * input + translation;
    return pcl::PointXYZ(output.x(), output.y(), output.z());
}

void ROSWrapper::shiftOccupiedCells(int shift_x, int shift_y)
{
    if (shift_x == 0 && shift_y == 0)
        return;

    for (auto& coord : occupied_global_cells)
    {
        coord.x += shift_x;
        coord.y += shift_y;
    }

    robot_coordinates_grid.x += shift_x;
    robot_coordinates_grid.y += shift_y;
}

void ROSWrapper::expandGlobalGridForCloud(const pcl::PointCloud<pcl::PointXYZ>& input)
{
    for (size_t i = 0; i < input.points.size(); i += static_cast<size_t>(cloud_point_stride))
    {
        const auto& point = input.points[i];
        GridExpansion expansion = grid_manager.expand_global_grid_to_include(
            point.x,
            point.y,
            global_map_growth_margin,
            global_map_growth_step,
            global_map_max_size);

        if (expansion.expanded)
        {
            shiftOccupiedCells(expansion.shift_x, expansion.shift_y);
            RCLCPP_INFO(this->get_logger(),
                        "Expanded global grid to %zu x %zu cells, origin=(%.2f, %.2f)",
                        grid_manager.global_grid[0].size(),
                        grid_manager.global_grid.size(),
                        grid_manager.global_origin_x,
                        grid_manager.global_origin_y);
        }
    }
}

void ROSWrapper::analyze_global_grid()
{
    TraversabilityStats stats;
    stats.occupied = static_cast<int>(occupied_global_cells.size());

    for (const auto& coord : occupied_global_cells)
    {
        TerrainCell& cell = grid_manager.global_grid[coord.y][coord.x];
        TerrainAnalysis analysis = t_score.AnalyzeCell(cell.points, analysis_config);

        cell.known = analysis.known;
        cell.traversable = analysis.traversable;
        cell.obstacle = analysis.obstacle;
        cell.floating_suppressed = false;
        cell.layer_class = static_cast<int>(analysis.layer_class);
        cell.floor_points = analysis.floor_points;
        cell.obstacle_points = analysis.obstacle_points;
        cell.ceiling_points = analysis.ceiling_points;
        cell.num_points = analysis.num_points;
        cell.slope = analysis.slope;
        cell.roughness = analysis.roughness;
        cell.height = analysis.height;
        cell.mean_z = analysis.mean_z;
        cell.z_min = analysis.z_min;
        cell.z_max = analysis.z_max;
        cell.z_p05 = analysis.z_p05;
        cell.z_p50 = analysis.z_p50;
        cell.z_p95 = analysis.z_p95;
        cell.confidence = analysis.confidence;
        cell.cost = analysis.cost;
    }

    stats.floating_suppressed = static_cast<int>(suppress_floating_floor_cells());
    compute_step_heights_for_occupied();

    for (const auto& coord : occupied_global_cells)
    {
        TerrainCell& cell = grid_manager.global_grid[coord.y][coord.x];
        if (cell.layer_class == static_cast<int>(CellLayerClass::Floor))
            ++stats.floor;
        else if (cell.layer_class == static_cast<int>(CellLayerClass::FloorWithObstacle))
            ++stats.floor_with_obstacle;
        else if (cell.layer_class == static_cast<int>(CellLayerClass::WallOrVerticalSurface))
            ++stats.wall;
        else if (cell.layer_class == static_cast<int>(CellLayerClass::CeilingOnly))
            ++stats.ceiling;

        if (!cell.known)
        {
            ++stats.unknown;
            continue;
        }

        ++stats.known;
        if (cell.obstacle)
        {
            cell.traversable = false;
            cell.cost = 100.0;
            ++stats.lethal_obstacle;
            continue;
        }

        const bool slope_ok = cell.slope <= analysis_config.max_traversable_slope;
        const bool roughness_ok = cell.roughness <= analysis_config.max_traversable_roughness;
        const bool height_ok = cell.height <= analysis_config.max_traversable_height;

        cell.traversable =
            slope_ok &&
            roughness_ok &&
            height_ok;
        cell.cost = t_score.calculateTScore(
            cell.slope, cell.roughness, cell.height, cell.confidence, cell.traversable, analysis_config);

        if (!cell.traversable)
        {
            if (!slope_ok)
                ++stats.lethal_slope;
            if (!roughness_ok)
                ++stats.lethal_roughness;
            if (!height_ok)
                ++stats.lethal_height;
        }
        else
        {
            ++stats.graded;
        }
    }

    apply_footprint_inflation(grid_manager.global_grid);
    RCLCPP_INFO(this->get_logger(),
                "Traversability stats: occupied=%d known=%d unknown=%d floor=%d floor_obstacle=%d wall=%d ceiling=%d floating=%d lethal_obstacle=%d lethal_slope=%d lethal_roughness=%d lethal_height=%d graded=%d",
                stats.occupied,
                stats.known,
                stats.unknown,
                stats.floor,
                stats.floor_with_obstacle,
                stats.wall,
                stats.ceiling,
                stats.floating_suppressed,
                stats.lethal_obstacle,
                stats.lethal_slope,
                stats.lethal_roughness,
                stats.lethal_height,
                stats.graded);
    map_dirty = false;
}

size_t ROSWrapper::suppress_floating_floor_cells()
{
    if (floating_floor_neighbor_radius <= 0 || max_floor_height_jump <= 0.0 ||
        grid_manager.global_grid.empty() || grid_manager.global_grid[0].empty())
        return 0;

    const int H = static_cast<int>(grid_manager.global_grid.size());
    const int W = static_cast<int>(grid_manager.global_grid[0].size());
    const int R = floating_floor_neighbor_radius;
    std::vector<GridCoord> floating_cells;

    for (const auto& coord : occupied_global_cells)
    {
        const TerrainCell& center = grid_manager.global_grid[coord.y][coord.x];
        if (!center.known || !center.traversable || center.cost >= 100.0)
            continue;

        double min_neighbor_floor = center.z_p50;
        bool has_lower_support = false;

        for (int dy = -R; dy <= R; ++dy)
        {
            const int y = coord.y + dy;
            if (y < 0 || y >= H)
                continue;

            for (int dx = -R; dx <= R; ++dx)
            {
                const int x = coord.x + dx;
                if (x < 0 || x >= W || (dx == 0 && dy == 0))
                    continue;

                const TerrainCell& nb = grid_manager.global_grid[y][x];
                if (!nb.known || nb.cost >= 100.0)
                    continue;

                min_neighbor_floor = std::min(min_neighbor_floor, nb.z_p50);
                if (center.z_p50 - nb.z_p50 > max_floor_height_jump)
                    has_lower_support = true;
            }
        }

        if (has_lower_support && center.z_p50 - min_neighbor_floor > max_floor_height_jump)
            floating_cells.push_back(coord);
    }

    for (const auto& coord : floating_cells)
    {
        TerrainCell& cell = grid_manager.global_grid[coord.y][coord.x];
        cell.known = false;
        cell.traversable = false;
        cell.floating_suppressed = true;
        cell.cost = -1.0;
    }

    return floating_cells.size();
}

void ROSWrapper::compute_step_heights_for_occupied()
{
    if (step_window_radius_cells <= 0 || grid_manager.global_grid.empty() || grid_manager.global_grid[0].empty())
        return;

    const int H = static_cast<int>(grid_manager.global_grid.size());
    const int W = static_cast<int>(grid_manager.global_grid[0].size());
    const int R = std::max(1, step_window_radius_cells);

    std::vector<double> updated_heights;
    updated_heights.reserve(occupied_global_cells.size());

    for (const auto& coord : occupied_global_cells)
    {
        const TerrainCell& center = grid_manager.global_grid[coord.y][coord.x];
        if (!center.known)
        {
            updated_heights.push_back(0.0);
            continue;
        }

        double max_dz = center.height;
        for (int dy = -R; dy <= R; ++dy)
        {
            const int y = coord.y + dy;
            if (y < 0 || y >= H)
                continue;

            for (int dx = -R; dx <= R; ++dx)
            {
                const int x = coord.x + dx;
                if (x < 0 || x >= W)
                    continue;

                const TerrainCell& nb = grid_manager.global_grid[y][x];
                if (!nb.known)
                    continue;

                max_dz = std::max(max_dz, std::abs(nb.z_p50 - center.z_p50));
            }
        }
        updated_heights.push_back(max_dz);
    }

    for (size_t i = 0; i < occupied_global_cells.size(); ++i)
    {
        const auto& coord = occupied_global_cells[i];
        grid_manager.global_grid[coord.y][coord.x].height = updated_heights[i];
    }
}

void ROSWrapper::update_local_grid_from_global()
{
    grid_manager.clear_grid(grid_manager.local_grid);

    if (!has_robot_pose)
        return;

    const int local_h = static_cast<int>(grid_manager.local_grid.size());
    const int local_w = static_cast<int>(grid_manager.local_grid[0].size());
    const int global_h = static_cast<int>(grid_manager.global_grid.size());
    const int global_w = static_cast<int>(grid_manager.global_grid[0].size());
    const int half_y = local_h / 2;
    const int half_x = local_w / 2;

    for (int ly = 0; ly < local_h; ++ly)
    {
        for (int lx = 0; lx < local_w; ++lx)
        {
            int gy = robot_coordinates_grid.y + (ly - half_y);
            int gx = robot_coordinates_grid.x + (lx - half_x);
            if (gy < 0 || gy >= global_h || gx < 0 || gx >= global_w)
                continue;
            grid_manager.local_grid[ly][lx] = grid_manager.global_grid[gy][gx];
        }
    }
}

void ROSWrapper::apply_footprint_inflation(TerrainGrid& grid)
{
    if (footprint_radius_cells <= 0 || grid.empty() || grid[0].empty())
        return;

    const int H = static_cast<int>(grid.size());
    const int W = static_cast<int>(grid[0].size());
    std::vector<GridCoord> lethal_cells;
    for (const auto& coord : occupied_global_cells)
    {
        const TerrainCell& cell = grid[coord.y][coord.x];
        if (cell.known && cell.cost >= 100.0)
            lethal_cells.push_back(coord);
    }

    for (const auto& coord : lethal_cells)
    {
        for (int dy = -footprint_radius_cells; dy <= footprint_radius_cells; ++dy)
        {
            for (int dx = -footprint_radius_cells; dx <= footprint_radius_cells; ++dx)
            {
                if (dx * dx + dy * dy > footprint_radius_cells * footprint_radius_cells)
                    continue;
                int y = coord.y + dy;
                int x = coord.x + dx;
                if (y < 0 || y >= H || x < 0 || x >= W)
                    continue;
                TerrainCell& cell = grid[y][x];
                if (!cell.known)
                    continue;
                cell.cost = std::max(cell.cost, 100.0);
                cell.traversable = false;
            }
        }
    }
}

nav_msgs::msg::OccupancyGrid ROSWrapper::make_occupancy_grid_message(const TerrainGrid& grid, bool is_local) const
{
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = map_frame_id;
    msg.info.resolution = resolution;

    const int H = static_cast<int>(grid.size());
    const int W = static_cast<int>(grid[0].size());
    msg.info.height = H;
    msg.info.width = W;

    const double size_x = W * resolution;
    const double size_y = H * resolution;
    if (is_local)
    {
        msg.info.origin.position.x = robot_coordinates[0] - size_x / 2.0;
        msg.info.origin.position.y = robot_coordinates[1] - size_y / 2.0;
    }
    else
    {
        msg.info.origin.position.x = grid_manager.global_origin_x;
        msg.info.origin.position.y = grid_manager.global_origin_y;
    }
    msg.info.origin.position.z = has_robot_pose ? robot_coordinates[2] : 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(W * H, -1);

    if (!is_local)
    {
        for (const auto& coord : occupied_global_cells)
        {
            if (coord.y < 0 || coord.y >= H || coord.x < 0 || coord.x >= W)
                continue;

            const TerrainCell& cell = grid[coord.y][coord.x];
            if (!cell.known)
                continue;
            msg.data[coord.y * W + coord.x] = static_cast<int8_t>(std::round(std::clamp(cell.cost, 0.0, 100.0)));
        }
        return msg;
    }

    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            const TerrainCell& cell = grid[y][x];
            if (cell.known)
                msg.data[y * W + x] = static_cast<int8_t>(std::round(std::clamp(cell.cost, 0.0, 100.0)));
        }
    }
    return msg;
}

nav_msgs::msg::OccupancyGrid ROSWrapper::make_metric_grid_message(const TerrainGrid& grid,
                                                                  bool is_local,
                                                                  const std::string& metric) const
{
    nav_msgs::msg::OccupancyGrid msg = make_occupancy_grid_message(grid, is_local);
    const int H = static_cast<int>(grid.size());
    const int W = static_cast<int>(grid[0].size());
    msg.data.assign(W * H, -1);

    if (!is_local)
    {
        for (const auto& coord : occupied_global_cells)
        {
            if (coord.y < 0 || coord.y >= H || coord.x < 0 || coord.x >= W)
                continue;

            const TerrainCell& cell = grid[coord.y][coord.x];
            if (!cell.known)
                continue;

            int value = 0;
            if (metric == "slope")
                value = scaleToOccupancy(cell.slope, analysis_config.slope_critical);
            else if (metric == "roughness")
                value = scaleToOccupancy(cell.roughness, analysis_config.roughness_critical);
            else if (metric == "height")
                value = scaleToOccupancy(cell.height, analysis_config.height_critical);
            else if (metric == "confidence")
                value = static_cast<int>(std::round(std::clamp(cell.confidence, 0.0, 1.0) * 100.0));

            msg.data[coord.y * W + coord.x] = static_cast<int8_t>(std::clamp(value, 0, 100));
        }
        return msg;
    }

    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            const TerrainCell& cell = grid[y][x];
            if (!cell.known)
                continue;

            int value = 0;
            if (metric == "slope")
                value = scaleToOccupancy(cell.slope, analysis_config.slope_critical);
            else if (metric == "roughness")
                value = scaleToOccupancy(cell.roughness, analysis_config.roughness_critical);
            else if (metric == "height")
                value = scaleToOccupancy(cell.height, analysis_config.height_critical);
            else if (metric == "confidence")
                value = static_cast<int>(std::round(std::clamp(cell.confidence, 0.0, 1.0) * 100.0));

            msg.data[y * W + x] = static_cast<int8_t>(std::clamp(value, 0, 100));
        }
    }
    return msg;
}

void ROSWrapper::updateAndPublish()
{
    if (!cloud_received)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "Waiting for point cloud before publishing traversability maps");
        return;
    }

    if (!updateRobotPose())
        return;

    const bool recomputed_global = map_dirty;
    if (map_dirty)
        compute_t_score();
    else
        update_local_grid_from_global();

    if (publish_local_map)
        publish_t_score_map(grid_manager.local_grid, true);

    if (recomputed_global || !publish_global_on_update_only)
    {
        publish_t_score_map(grid_manager.global_grid, false);
        publish_debug_maps_for_grid(grid_manager.global_grid, false);
    }

    if (one_shot)
    {
        RCLCPP_INFO(this->get_logger(), "One-shot traversability map published; shutting down");
        rclcpp::shutdown();
    }
}




// =====================================================
// TOOLS
// =====================================================


// Utility method to get parameters from the config file
void ROSWrapper::get_parameters(std::string parameters_path)
{
    // Open the file for reading 
    FILE* fp = fopen(parameters_path.c_str(), "r");
    
    // Use a FileReadStream to 
    // read the data from the file 
    char readBuffer[65536]; 
    rapidjson::FileReadStream is(fp, readBuffer, 
                                 sizeof(readBuffer)); 
  
    // Parse the JSON data  
    // using a Document object 
    // rapidjson::Document d; 
    p.ParseStream(is); 
  
    // Close the file 
    fclose(fp); 
};










int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSWrapper>());
  rclcpp::shutdown();
  return 0;
}
