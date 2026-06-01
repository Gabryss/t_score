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

const rapidjson::Value* findProfile(const rapidjson::Document& doc, const std::string& profile_name)
{
    if (profile_name.empty() || !doc.HasMember("traversability_profiles") || !doc["traversability_profiles"].IsObject())
        return nullptr;
    const auto& profiles = doc["traversability_profiles"];
    if (!profiles.HasMember(profile_name.c_str()) || !profiles[profile_name.c_str()].IsObject())
        return nullptr;
    return &profiles[profile_name.c_str()];
}

double getProfileDouble(const rapidjson::Document& doc, const rapidjson::Value* profile, const char* key, double fallback)
{
    if (profile != nullptr && profile->HasMember(key) && (*profile)[key].IsNumber())
        return (*profile)[key].GetDouble();
    return getDouble(doc, key, fallback);
}

int getProfileInt(const rapidjson::Document& doc, const rapidjson::Value* profile, const char* key, int fallback)
{
    if (profile != nullptr && profile->HasMember(key) && (*profile)[key].IsInt())
        return (*profile)[key].GetInt();
    return getInt(doc, key, fallback);
}

bool getProfileBool(const rapidjson::Document& doc, const rapidjson::Value* profile, const char* key, bool fallback)
{
    if (profile != nullptr && profile->HasMember(key) && (*profile)[key].IsBool())
        return (*profile)[key].GetBool();
    return getBool(doc, key, fallback);
}

std::string getProfileString(const rapidjson::Document& doc, const rapidjson::Value* profile, const char* key, const std::string& fallback)
{
    if (profile != nullptr && profile->HasMember(key) && (*profile)[key].IsString())
        return (*profile)[key].GetString();
    return getString(doc, key, fallback);
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
    int unsupported_floor = 0;
    int multires_filled = 0;
    int second_multires_filled = 0;
    int neighbor_gap_filled = 0;
    int unknown_region_filled = 0;
    int cost_smoothed = 0;
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

    this->declare_parameter("traversability_profile", getString(p, "traversability_profile", ""));
    const std::string profile_name = this->get_parameter("traversability_profile").as_string();
    const rapidjson::Value* active_profile = findProfile(p, profile_name);
    if (!profile_name.empty())
    {
        if (active_profile != nullptr)
            RCLCPP_INFO(this->get_logger(), "Using traversability profile '%s'", profile_name.c_str());
        else
            RCLCPP_WARN(this->get_logger(), "Traversability profile '%s' not found; using base parameters", profile_name.c_str());
    }

    auto cfg_double = [&](const char* key, double fallback) {
        return getProfileDouble(p, active_profile, key, fallback);
    };
    auto cfg_int = [&](const char* key, int fallback) {
        return getProfileInt(p, active_profile, key, fallback);
    };
    auto cfg_bool = [&](const char* key, bool fallback) {
        return getProfileBool(p, active_profile, key, fallback);
    };
    auto cfg_string = [&](const char* key, const std::string& fallback) {
        return getProfileString(p, active_profile, key, fallback);
    };
    auto param_double = [&](const char* key, double fallback) {
        return this->declare_parameter<double>(key, cfg_double(key, fallback));
    };
    auto param_int = [&](const char* key, int fallback) {
        return static_cast<int>(this->declare_parameter<int>(key, cfg_int(key, fallback)));
    };
    auto param_bool = [&](const char* key, bool fallback) {
        return this->declare_parameter<bool>(key, cfg_bool(key, fallback));
    };
    auto param_string = [&](const char* key, const std::string& fallback) {
        return this->declare_parameter<std::string>(key, cfg_string(key, fallback));
    };

    resolution = param_double("map_resolution", 0.5);
    local_map_size = param_double("local_map_size", 5.0);
    global_map_size = param_double("global_map_size", 1000.0);

    update_frequency = param_double("update_frequency", 1.0);
    int update_period_ms = static_cast<int>(1000.0 / std::max(0.1f, update_frequency));
    map_frame_id = param_string("traversability_frame_id", "map");
    robot_frame_id = param_string("robot_frame_id", "base_footprint");
    const std::string ros_namespace = normalizeNamespace(param_string("ros_namespace", ""));
    debug_logging = param_bool("debug_logging", false);
    publish_debug_maps = param_bool("publish_debug_maps", true);
    rebuild_global_map_on_cloud = param_bool("rebuild_global_map_on_cloud", true);
    compute_on_cloud_update = param_bool("compute_on_cloud_update", false);
    publish_on_timer = param_bool("publish_on_timer", true);
    one_shot = param_bool("one_shot", false);
    allow_identity_pose_fallback = param_bool("allow_identity_pose_fallback", true);
    publish_global_on_update_only = param_bool("publish_global_on_update_only", true);
    publish_local_map = param_bool("publish_local_map", true);
    enable_footprint_inflation = param_bool("enable_footprint_inflation", enable_footprint_inflation);
    step_window_radius_cells = param_int("step_window_radius_cells", 1);
    max_points_per_cell = std::max(3, param_int("max_points_per_cell", 80));
    cloud_point_stride = std::max(1, param_int("cloud_point_stride", 1));
    robot_radius = param_double("robot_radius", 0.45);
    global_map_growth_margin = param_double("global_map_growth_margin", 5.0);
    global_map_growth_step = param_double("global_map_growth_step", 20.0);
    global_map_max_size = param_double("global_map_max_size", std::max<double>(global_map_size, 300.0));
    floating_floor_neighbor_radius = param_int("floating_floor_neighbor_radius", floating_floor_neighbor_radius);
    max_floor_height_jump = param_double("max_floor_height_jump", max_floor_height_jump);
    enable_global_floor_support = param_bool("enable_global_floor_support", enable_global_floor_support);
    global_floor_seed_quantile = param_double("global_floor_seed_quantile", global_floor_seed_quantile);
    global_floor_seed_height = param_double("global_floor_seed_height", global_floor_seed_height);
    global_floor_max_step = param_double("global_floor_max_step", global_floor_max_step);
    enable_multires_unknown_fill = param_bool("enable_multires_unknown_fill", enable_multires_unknown_fill);
    multires_fill_resolution = param_double("multires_fill_resolution", multires_fill_resolution);
    multires_fill_min_known = param_int("multires_fill_min_known", multires_fill_min_known);
    enable_second_multires_unknown_fill = param_bool("enable_second_multires_unknown_fill", enable_second_multires_unknown_fill);
    second_multires_fill_resolution = param_double("second_multires_fill_resolution", second_multires_fill_resolution);
    second_multires_fill_min_known = param_int("second_multires_fill_min_known", second_multires_fill_min_known);
    enable_neighbor_gap_fill = param_bool("enable_neighbor_gap_fill", enable_neighbor_gap_fill);
    neighbor_gap_fill_min_neighbors = param_int("neighbor_gap_fill_min_neighbors", neighbor_gap_fill_min_neighbors);
    neighbor_gap_fill_cost_penalty = param_double("neighbor_gap_fill_cost_penalty", neighbor_gap_fill_cost_penalty);
    neighbor_gap_fill_skip_if_blocked_neighbor = param_bool(
        "neighbor_gap_fill_skip_if_blocked_neighbor",
        neighbor_gap_fill_skip_if_blocked_neighbor);
    enable_unknown_region_fill = param_bool("enable_unknown_region_fill", enable_unknown_region_fill);
    unknown_region_fill_max_cells = param_int("unknown_region_fill_max_cells", unknown_region_fill_max_cells);
    unknown_region_fill_min_boundary_known = param_int(
        "unknown_region_fill_min_boundary_known",
        unknown_region_fill_min_boundary_known);
    unknown_region_fill_max_wall_fraction = param_double(
        "unknown_region_fill_max_wall_fraction",
        unknown_region_fill_max_wall_fraction);
    unknown_region_fill_max_blocked_fraction = param_double(
        "unknown_region_fill_max_blocked_fraction",
        unknown_region_fill_max_blocked_fraction);
    unknown_region_fill_cost_penalty = param_double(
        "unknown_region_fill_cost_penalty",
        unknown_region_fill_cost_penalty);
    enable_cost_smoothing = param_bool("enable_cost_smoothing", enable_cost_smoothing);
    cost_smoothing_iterations = param_int("cost_smoothing_iterations", cost_smoothing_iterations);
    cost_smoothing_radius = param_int("cost_smoothing_radius", cost_smoothing_radius);
    cost_smoothing_neighbor_weight = param_double(
        "cost_smoothing_neighbor_weight",
        cost_smoothing_neighbor_weight);
    cost_smoothing_max_cost_delta = param_double(
        "cost_smoothing_max_cost_delta",
        cost_smoothing_max_cost_delta);
    footprint_radius_cells = enable_footprint_inflation
        ? std::max(1, static_cast<int>(std::ceil(robot_radius / resolution)))
        : 0;

    analysis_config.min_points = param_int("min_points_per_cell", analysis_config.min_points);
    analysis_config.confidence_full_points = param_int("confidence_full_points", analysis_config.confidence_full_points);
    analysis_config.slope_critical = param_double("slope_critical", analysis_config.slope_critical);
    analysis_config.roughness_critical = param_double("roughness_critical", analysis_config.roughness_critical);
    analysis_config.height_critical = param_double("height_critical", analysis_config.height_critical);
    analysis_config.max_traversable_slope = param_double("max_traversable_slope", analysis_config.max_traversable_slope);
    analysis_config.max_traversable_roughness = param_double("max_traversable_roughness", analysis_config.max_traversable_roughness);
    analysis_config.max_traversable_height = param_double("max_traversable_height", analysis_config.max_traversable_height);
    analysis_config.slope_weight = param_double("slope_weight", analysis_config.slope_weight);
    analysis_config.roughness_weight = param_double("roughness_weight", analysis_config.roughness_weight);
    analysis_config.height_weight = param_double("height_weight", analysis_config.height_weight);
    analysis_config.confidence_weight = param_double("confidence_weight", analysis_config.confidence_weight);
    analysis_config.enable_ground_layer_filter = param_bool("enable_ground_layer_filter", analysis_config.enable_ground_layer_filter);
    analysis_config.ground_quantile = param_double("ground_quantile", analysis_config.ground_quantile);
    analysis_config.ground_band_below = param_double("ground_band_below", analysis_config.ground_band_below);
    analysis_config.ground_band_above = param_double("ground_band_above", analysis_config.ground_band_above);
    analysis_config.ceiling_ignore_height = param_double("ceiling_ignore_height", analysis_config.ceiling_ignore_height);
    analysis_config.obstacle_min_height = param_double("obstacle_min_height", analysis_config.obstacle_min_height);
    analysis_config.obstacle_min_points = param_int("obstacle_min_points", analysis_config.obstacle_min_points);
    analysis_config.enable_column_classifier = param_bool("enable_column_classifier", analysis_config.enable_column_classifier);
    analysis_config.min_floor_points = param_int("min_floor_points", analysis_config.min_floor_points);
    analysis_config.ceiling_min_points = param_int("ceiling_min_points", analysis_config.ceiling_min_points);
    analysis_config.wall_min_vertical_span = param_double("wall_min_vertical_span", analysis_config.wall_min_vertical_span);
    analysis_config.wall_min_points = param_int("wall_min_points", analysis_config.wall_min_points);
    analysis_config.wall_cells_as_obstacles = param_bool("wall_cells_as_obstacles", analysis_config.wall_cells_as_obstacles);
    analysis_config.require_floor_for_obstacle = param_bool("require_floor_for_obstacle", analysis_config.require_floor_for_obstacle);


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
    rclcpp::QoS pc_qos(param_int("pc_qos_depth", 1));
    pc_qos.reliable();
    if (param_bool("pc_qos_transient_local", true))
        pc_qos.transient_local();
    else
        pc_qos.durability_volatile();

    sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
     resolveTopicName(param_string("pc_topic", "/rtabmap/cloud_map"), ros_namespace),
     pc_qos,
     std::bind(&ROSWrapper::pc_callback, this, _1));
    

    // Create publishers
    pub_t_score_local_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        resolveTopicName(param_string("traversability_topic_local", "/traversability_costmap_local"), ros_namespace), 10);
    pub_t_score_global_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        resolveTopicName(param_string("traversability_topic_global", "/traversability_costmap"), ros_namespace), 10);

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
    stats.unsupported_floor = static_cast<int>(apply_global_floor_support());
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

    if (enable_multires_unknown_fill)
    {
        stats.multires_filled = static_cast<int>(
            fill_unknown_cells_from_coarse_blocks(multires_fill_resolution, multires_fill_min_known, false));

        if (enable_second_multires_unknown_fill)
        {
            stats.second_multires_filled = static_cast<int>(
                fill_unknown_cells_from_coarse_blocks(
                    second_multires_fill_resolution,
                    second_multires_fill_min_known,
                    true));
        }
    }
    stats.neighbor_gap_filled = static_cast<int>(fill_unknown_cells_from_neighbors());
    stats.unknown_region_filled = static_cast<int>(fill_small_unknown_regions());
    stats.cost_smoothed = static_cast<int>(smooth_traversability_costs());
    apply_footprint_inflation(grid_manager.global_grid);
    RCLCPP_INFO(this->get_logger(),
                "Traversability stats: occupied=%d known=%d unknown=%d floor=%d floor_obstacle=%d wall=%d ceiling=%d floating=%d unsupported_floor=%d multires_filled=%d second_multires_filled=%d neighbor_gap_filled=%d unknown_region_filled=%d cost_smoothed=%d lethal_obstacle=%d lethal_slope=%d lethal_roughness=%d lethal_height=%d graded=%d",
                stats.occupied,
                stats.known,
                stats.unknown,
                stats.floor,
                stats.floor_with_obstacle,
                stats.wall,
                stats.ceiling,
                stats.floating_suppressed,
                stats.unsupported_floor,
                stats.multires_filled,
                stats.second_multires_filled,
                stats.neighbor_gap_filled,
                stats.unknown_region_filled,
                stats.cost_smoothed,
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

size_t ROSWrapper::apply_global_floor_support()
{
    if (!enable_global_floor_support || global_floor_max_step <= 0.0 ||
        grid_manager.global_grid.empty() || grid_manager.global_grid[0].empty())
        return 0;

    std::vector<double> floor_heights;
    floor_heights.reserve(occupied_global_cells.size());
    for (const auto& coord : occupied_global_cells)
    {
        const TerrainCell& cell = grid_manager.global_grid[coord.y][coord.x];
        if (cell.known &&
            !cell.obstacle &&
            cell.layer_class == static_cast<int>(CellLayerClass::Floor))
        {
            floor_heights.push_back(cell.z_p50);
        }
    }

    if (floor_heights.empty())
        return 0;

    std::sort(floor_heights.begin(), floor_heights.end());
    const double q = std::clamp(global_floor_seed_quantile, 0.0, 1.0);
    const size_t seed_index = static_cast<size_t>(std::round(q * static_cast<double>(floor_heights.size() - 1)));
    const double seed_ceiling = floor_heights[seed_index] + std::max(0.0, global_floor_seed_height);

    const int H = static_cast<int>(grid_manager.global_grid.size());
    const int W = static_cast<int>(grid_manager.global_grid[0].size());
    std::vector<uint8_t> supported(static_cast<size_t>(W) * static_cast<size_t>(H), 0);
    std::deque<GridCoord> queue;

    auto index_of = [W](int x, int y) {
        return static_cast<size_t>(y) * static_cast<size_t>(W) + static_cast<size_t>(x);
    };

    for (const auto& coord : occupied_global_cells)
    {
        const TerrainCell& cell = grid_manager.global_grid[coord.y][coord.x];
        if (cell.known &&
            !cell.obstacle &&
            cell.layer_class == static_cast<int>(CellLayerClass::Floor) &&
            cell.z_p50 <= seed_ceiling)
        {
            supported[index_of(coord.x, coord.y)] = 1;
            queue.push_back(coord);
        }
    }

    while (!queue.empty())
    {
        const GridCoord current = queue.front();
        queue.pop_front();
        const TerrainCell& current_cell = grid_manager.global_grid[current.y][current.x];

        for (int dy = -1; dy <= 1; ++dy)
        {
            const int y = current.y + dy;
            if (y < 0 || y >= H)
                continue;

            for (int dx = -1; dx <= 1; ++dx)
            {
                const int x = current.x + dx;
                if (x < 0 || x >= W || (dx == 0 && dy == 0))
                    continue;

                const size_t idx = index_of(x, y);
                if (supported[idx])
                    continue;

                const TerrainCell& nb = grid_manager.global_grid[y][x];
                if (!nb.known ||
                    nb.obstacle ||
                    nb.layer_class != static_cast<int>(CellLayerClass::Floor))
                    continue;

                if (std::abs(nb.z_p50 - current_cell.z_p50) > global_floor_max_step)
                    continue;

                supported[idx] = 1;
                queue.push_back({x, y});
            }
        }
    }

    size_t suppressed_count = 0;
    for (const auto& coord : occupied_global_cells)
    {
        TerrainCell& cell = grid_manager.global_grid[coord.y][coord.x];
        if (!cell.known ||
            cell.obstacle ||
            cell.layer_class != static_cast<int>(CellLayerClass::Floor))
            continue;

        if (supported[index_of(coord.x, coord.y)])
            continue;

        cell.known = false;
        cell.traversable = false;
        cell.floating_suppressed = true;
        cell.cost = -1.0;
        ++suppressed_count;
    }

    return suppressed_count;
}

size_t ROSWrapper::fill_unknown_cells_from_coarse_blocks(
    double fill_resolution,
    int min_known,
    bool skip_lethal_blocks)
{
    if (fill_resolution <= resolution ||
        grid_manager.global_grid.empty() ||
        grid_manager.global_grid[0].empty())
        return 0;

    const int H = static_cast<int>(grid_manager.global_grid.size());
    const int W = static_cast<int>(grid_manager.global_grid[0].size());
    const int block_size = std::max(2, static_cast<int>(std::round(fill_resolution / resolution)));
    const int required_known = std::max(1, min_known);

    std::unordered_set<int64_t> occupied_ids;
    occupied_ids.reserve(occupied_global_cells.size() * 2 + 1);
    for (const auto& coord : occupied_global_cells)
        occupied_ids.insert(static_cast<int64_t>(coord.y) * static_cast<int64_t>(W) + coord.x);

    size_t filled_count = 0;

    for (int y0 = 0; y0 < H; y0 += block_size)
    {
        for (int x0 = 0; x0 < W; x0 += block_size)
        {
            int known_count = 0;
            double cost_sum = 0.0;
            double confidence_sum = 0.0;
            double slope_sum = 0.0;
            double roughness_sum = 0.0;
            double height_sum = 0.0;
            double z_sum = 0.0;
            bool any_lethal = false;
            bool any_traversable = false;

            const int y_end = std::min(H, y0 + block_size);
            const int x_end = std::min(W, x0 + block_size);

            for (int y = y0; y < y_end; ++y)
            {
                for (int x = x0; x < x_end; ++x)
                {
                    const TerrainCell& cell = grid_manager.global_grid[y][x];
                    if (!cell.known || cell.cost < 0.0)
                        continue;

                    ++known_count;
                    cost_sum += std::clamp(cell.cost, 0.0, 100.0);
                    confidence_sum += cell.confidence;
                    slope_sum += cell.slope;
                    roughness_sum += cell.roughness;
                    height_sum += cell.height;
                    z_sum += cell.z_p50;
                    any_lethal = any_lethal || cell.cost >= 100.0;
                    any_traversable = any_traversable || cell.traversable;
                }
            }

            if (known_count < required_known || (skip_lethal_blocks && any_lethal))
                continue;

            const double inv_count = 1.0 / static_cast<double>(known_count);
            const double filled_cost = std::round(cost_sum * inv_count);
            const double filled_confidence = std::clamp(confidence_sum * inv_count, 0.0, 1.0);
            const double filled_slope = slope_sum * inv_count;
            const double filled_roughness = roughness_sum * inv_count;
            const double filled_height = height_sum * inv_count;
            const double filled_z = z_sum * inv_count;

            for (int y = y0; y < y_end; ++y)
            {
                for (int x = x0; x < x_end; ++x)
                {
                    TerrainCell& cell = grid_manager.global_grid[y][x];
                    if (cell.known)
                        continue;

                    cell.known = true;
                    cell.traversable = any_traversable && !any_lethal && filled_cost < 100.0;
                    cell.obstacle = false;
                    cell.cost = std::clamp(filled_cost, 0.0, 100.0);
                    cell.confidence = filled_confidence;
                    cell.slope = filled_slope;
                    cell.roughness = filled_roughness;
                    cell.height = filled_height;
                    cell.mean_z = filled_z;
                    cell.z_min = filled_z;
                    cell.z_max = filled_z;
                    cell.z_p05 = filled_z;
                    cell.z_p50 = filled_z;
                    cell.z_p95 = filled_z;

                    const int64_t id = static_cast<int64_t>(y) * static_cast<int64_t>(W) + x;
                    if (occupied_ids.insert(id).second)
                        occupied_global_cells.push_back({x, y});
                    ++filled_count;
                }
            }
        }
    }

    return filled_count;
}

size_t ROSWrapper::fill_unknown_cells_from_neighbors()
{
    if (!enable_neighbor_gap_fill ||
        grid_manager.global_grid.empty() ||
        grid_manager.global_grid[0].empty())
        return 0;

    const int H = static_cast<int>(grid_manager.global_grid.size());
    const int W = static_cast<int>(grid_manager.global_grid[0].size());
    const int min_neighbors = std::clamp(neighbor_gap_fill_min_neighbors, 1, 8);
    const TerrainGrid source = grid_manager.global_grid;

    std::unordered_set<int64_t> occupied_ids;
    occupied_ids.reserve(occupied_global_cells.size() * 2 + 1);
    for (const auto& coord : occupied_global_cells)
        occupied_ids.insert(static_cast<int64_t>(coord.y) * static_cast<int64_t>(W) + coord.x);

    size_t filled_count = 0;

    for (int y = 1; y < H - 1; ++y)
    {
        for (int x = 1; x < W - 1; ++x)
        {
            TerrainCell& target = grid_manager.global_grid[y][x];
            if (target.known)
                continue;

            int valid_neighbors = 0;
            bool blocked_neighbor = false;
            double cost_sum = 0.0;
            double confidence_sum = 0.0;
            double slope_sum = 0.0;
            double roughness_sum = 0.0;
            double height_sum = 0.0;
            double z_sum = 0.0;
            bool any_obstacle = false;

            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dx = -1; dx <= 1; ++dx)
                {
                    if (dx == 0 && dy == 0)
                        continue;

                    const TerrainCell& nb = source[y + dy][x + dx];
                    const bool is_wall =
                        nb.layer_class == static_cast<int>(CellLayerClass::WallOrVerticalSurface);
                    const bool is_blocked =
                        !nb.known ||
                        is_wall ||
                        nb.cost < 0.0;

                    if (is_blocked)
                    {
                        blocked_neighbor = true;
                        continue;
                    }

                    ++valid_neighbors;
                    cost_sum += std::clamp(nb.cost, 0.0, 100.0);
                    confidence_sum += nb.confidence;
                    slope_sum += nb.slope;
                    roughness_sum += nb.roughness;
                    height_sum += nb.height;
                    z_sum += nb.z_p50;
                    any_obstacle = any_obstacle || nb.obstacle;
                }
            }

            if (valid_neighbors < min_neighbors ||
                (neighbor_gap_fill_skip_if_blocked_neighbor && blocked_neighbor))
                continue;

            const double inv_count = 1.0 / static_cast<double>(valid_neighbors);
            const double filled_cost = std::clamp(
                std::round(cost_sum * inv_count + neighbor_gap_fill_cost_penalty),
                0.0,
                100.0);
            const double filled_confidence = std::clamp(confidence_sum * inv_count * 0.75, 0.0, 1.0);
            const double filled_slope = slope_sum * inv_count;
            const double filled_roughness = roughness_sum * inv_count;
            const double filled_height = height_sum * inv_count;
            const double filled_z = z_sum * inv_count;

            target.known = true;
            target.traversable = filled_cost < 100.0;
            target.obstacle = any_obstacle || filled_cost >= 100.0;
            target.cost = filled_cost;
            target.confidence = filled_confidence;
            target.slope = filled_slope;
            target.roughness = filled_roughness;
            target.height = filled_height;
            target.mean_z = filled_z;
            target.z_min = filled_z;
            target.z_max = filled_z;
            target.z_p05 = filled_z;
            target.z_p50 = filled_z;
            target.z_p95 = filled_z;
            target.layer_class = static_cast<int>(CellLayerClass::Floor);

            const int64_t id = static_cast<int64_t>(y) * static_cast<int64_t>(W) + x;
            if (occupied_ids.insert(id).second)
                occupied_global_cells.push_back({x, y});
            ++filled_count;
        }
    }

    return filled_count;
}

size_t ROSWrapper::fill_small_unknown_regions()
{
    if (!enable_unknown_region_fill ||
        grid_manager.global_grid.empty() ||
        grid_manager.global_grid[0].empty())
        return 0;

    const int H = static_cast<int>(grid_manager.global_grid.size());
    const int W = static_cast<int>(grid_manager.global_grid[0].size());
    const int max_region_cells = std::max(1, unknown_region_fill_max_cells);
    const int min_boundary_known = std::max(1, unknown_region_fill_min_boundary_known);
    const double max_wall_fraction = std::clamp(unknown_region_fill_max_wall_fraction, 0.0, 1.0);
    const double max_blocked_fraction = std::clamp(unknown_region_fill_max_blocked_fraction, 0.0, 1.0);
    const TerrainGrid source = grid_manager.global_grid;

    std::vector<uint8_t> visited(static_cast<size_t>(H) * static_cast<size_t>(W), 0);
    std::unordered_set<int64_t> occupied_ids;
    occupied_ids.reserve(occupied_global_cells.size() * 2 + 1);
    for (const auto& coord : occupied_global_cells)
        occupied_ids.insert(static_cast<int64_t>(coord.y) * static_cast<int64_t>(W) + coord.x);

    auto index_of = [W](int x, int y) {
        return y * W + x;
    };

    size_t filled_count = 0;

    for (int start_y = 0; start_y < H; ++start_y)
    {
        for (int start_x = 0; start_x < W; ++start_x)
        {
            const int start_index = index_of(start_x, start_y);
            if (visited[start_index] || source[start_y][start_x].known)
                continue;

            std::vector<GridCoord> region;
            std::deque<GridCoord> queue;
            bool touches_border = false;
            visited[start_index] = 1;
            queue.push_back({start_x, start_y});

            while (!queue.empty())
            {
                const GridCoord current = queue.front();
                queue.pop_front();
                region.push_back(current);
                touches_border =
                    touches_border ||
                    current.x == 0 ||
                    current.y == 0 ||
                    current.x == W - 1 ||
                    current.y == H - 1;

                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        if (dx == 0 && dy == 0)
                            continue;

                        const int nx = current.x + dx;
                        const int ny = current.y + dy;
                        if (nx < 0 || nx >= W || ny < 0 || ny >= H)
                            continue;

                        const int ni = index_of(nx, ny);
                        if (visited[ni] || source[ny][nx].known)
                            continue;

                        visited[ni] = 1;
                        queue.push_back({nx, ny});
                    }
                }
            }

            if (touches_border || static_cast<int>(region.size()) > max_region_cells)
                continue;

            std::unordered_set<int64_t> boundary_ids;
            boundary_ids.reserve(region.size() * 4);
            for (const auto& cell_coord : region)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        if (dx == 0 && dy == 0)
                            continue;

                        const int nx = cell_coord.x + dx;
                        const int ny = cell_coord.y + dy;
                        if (nx < 0 || nx >= W || ny < 0 || ny >= H)
                            continue;

                        const TerrainCell& nb = source[ny][nx];
                        if (!nb.known || nb.cost < 0.0)
                            continue;

                        boundary_ids.insert(static_cast<int64_t>(ny) * static_cast<int64_t>(W) + nx);
                    }
                }
            }

            if (static_cast<int>(boundary_ids.size()) < min_boundary_known)
                continue;

            int wall_count = 0;
            int blocked_count = 0;
            double cost_sum = 0.0;
            double confidence_sum = 0.0;
            double slope_sum = 0.0;
            double roughness_sum = 0.0;
            double height_sum = 0.0;
            double z_sum = 0.0;

            for (const int64_t id : boundary_ids)
            {
                const int x = static_cast<int>(id % W);
                const int y = static_cast<int>(id / W);
                const TerrainCell& boundary = source[y][x];
                const bool is_wall =
                    boundary.layer_class == static_cast<int>(CellLayerClass::WallOrVerticalSurface);
                const bool is_blocked =
                    is_wall ||
                    boundary.obstacle ||
                    boundary.cost >= 100.0;

                wall_count += is_wall ? 1 : 0;
                blocked_count += is_blocked ? 1 : 0;
                cost_sum += std::clamp(boundary.cost, 0.0, 100.0);
                confidence_sum += boundary.confidence;
                slope_sum += boundary.slope;
                roughness_sum += boundary.roughness;
                height_sum += boundary.height;
                z_sum += boundary.z_p50;
            }

            const double inv_boundary = 1.0 / static_cast<double>(boundary_ids.size());
            const double wall_fraction = static_cast<double>(wall_count) * inv_boundary;
            const double blocked_fraction = static_cast<double>(blocked_count) * inv_boundary;
            if (wall_fraction > max_wall_fraction || blocked_fraction > max_blocked_fraction)
                continue;

            const double filled_cost = std::clamp(
                std::round(cost_sum * inv_boundary + unknown_region_fill_cost_penalty),
                0.0,
                100.0);
            const double filled_confidence = std::clamp(confidence_sum * inv_boundary * 0.65, 0.0, 1.0);
            const double filled_slope = slope_sum * inv_boundary;
            const double filled_roughness = roughness_sum * inv_boundary;
            const double filled_height = height_sum * inv_boundary;
            const double filled_z = z_sum * inv_boundary;

            for (const auto& cell_coord : region)
            {
                TerrainCell& target = grid_manager.global_grid[cell_coord.y][cell_coord.x];
                if (target.known)
                    continue;

                target.known = true;
                target.traversable = filled_cost < 100.0;
                target.obstacle = filled_cost >= 100.0;
                target.cost = filled_cost;
                target.confidence = filled_confidence;
                target.slope = filled_slope;
                target.roughness = filled_roughness;
                target.height = filled_height;
                target.mean_z = filled_z;
                target.z_min = filled_z;
                target.z_max = filled_z;
                target.z_p05 = filled_z;
                target.z_p50 = filled_z;
                target.z_p95 = filled_z;
                target.layer_class = static_cast<int>(CellLayerClass::Floor);

                const int64_t id = static_cast<int64_t>(cell_coord.y) * static_cast<int64_t>(W) + cell_coord.x;
                if (occupied_ids.insert(id).second)
                    occupied_global_cells.push_back(cell_coord);
                ++filled_count;
            }
        }
    }

    return filled_count;
}

size_t ROSWrapper::smooth_traversability_costs()
{
    if (!enable_cost_smoothing ||
        grid_manager.global_grid.empty() ||
        grid_manager.global_grid[0].empty())
        return 0;

    const int H = static_cast<int>(grid_manager.global_grid.size());
    const int W = static_cast<int>(grid_manager.global_grid[0].size());
    const int iterations = std::max(0, cost_smoothing_iterations);
    const int radius = std::max(1, cost_smoothing_radius);
    const double neighbor_weight = std::clamp(cost_smoothing_neighbor_weight, 0.0, 1.0);
    const double max_delta = std::max(0.0, cost_smoothing_max_cost_delta);

    if (iterations == 0 || neighbor_weight <= 0.0)
        return 0;

    auto is_smoothable = [](const TerrainCell& cell) {
        return cell.known &&
               cell.cost >= 0.0 &&
               cell.cost < 100.0 &&
               !cell.obstacle &&
               cell.layer_class != static_cast<int>(CellLayerClass::WallOrVerticalSurface) &&
               cell.layer_class != static_cast<int>(CellLayerClass::CeilingOnly);
    };

    size_t changed_count = 0;

    for (int iter = 0; iter < iterations; ++iter)
    {
        const TerrainGrid source = grid_manager.global_grid;
        std::vector<double> smoothed_costs(static_cast<size_t>(H) * static_cast<size_t>(W), -1.0);

        for (int y = 0; y < H; ++y)
        {
            for (int x = 0; x < W; ++x)
            {
                const TerrainCell& center = source[y][x];
                if (!is_smoothable(center))
                    continue;

                int neighbor_count = 0;
                double neighbor_cost_sum = 0.0;

                for (int dy = -radius; dy <= radius; ++dy)
                {
                    const int ny = y + dy;
                    if (ny < 0 || ny >= H)
                        continue;

                    for (int dx = -radius; dx <= radius; ++dx)
                    {
                        const int nx = x + dx;
                        if ((dx == 0 && dy == 0) || nx < 0 || nx >= W)
                            continue;
                        if (dx * dx + dy * dy > radius * radius)
                            continue;

                        const TerrainCell& nb = source[ny][nx];
                        if (!is_smoothable(nb))
                            continue;
                        if (std::abs(nb.cost - center.cost) > max_delta)
                            continue;

                        neighbor_cost_sum += nb.cost;
                        ++neighbor_count;
                    }
                }

                if (neighbor_count == 0)
                    continue;

                const double neighbor_mean = neighbor_cost_sum / static_cast<double>(neighbor_count);
                const double new_cost = std::clamp(
                    std::round(center.cost * (1.0 - neighbor_weight) + neighbor_mean * neighbor_weight),
                    0.0,
                    99.0);
                smoothed_costs[static_cast<size_t>(y) * static_cast<size_t>(W) + x] = new_cost;
            }
        }

        for (int y = 0; y < H; ++y)
        {
            for (int x = 0; x < W; ++x)
            {
                const double new_cost = smoothed_costs[static_cast<size_t>(y) * static_cast<size_t>(W) + x];
                if (new_cost < 0.0)
                    continue;

                TerrainCell& cell = grid_manager.global_grid[y][x];
                if (std::abs(cell.cost - new_cost) >= 0.5)
                    ++changed_count;
                cell.cost = new_cost;
                cell.traversable = cell.cost < 100.0;
            }
        }
    }

    return changed_count;
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
