/**
 * @file ROSWrapper.hpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2025-11-19
 * 
 * @copyright Gabriel Garcia | 2025
 * @brief Implementation file for Class ROSWrapper.
 * @details This is a ROS wrapper that subscribe to a pointcloud and publish a traversability costmap.
 */
#include <memory>
#include "iostream"
#include <fstream>
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <functional>
#include <boost/make_shared.hpp>
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <deque>
#include <vector>
#include <unordered_set>


//ROS
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/create_timer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>



// Custom library
#include "GridManager.hpp"
#include "TScore.hpp"


using std::placeholders::_1;
using std::vector;
using namespace std::chrono_literals;



// Coordinates
using coordinates = vector<double>;

using coordinates_grid = vector<int>;


class ROSWrapper : public rclcpp::Node
{
    public:
        ROSWrapper();
        ~ROSWrapper();

        // ===========================
        // Attributes
        // ===========================

        GridManager grid_manager;
        TScore t_score;

        double height_thresh = 0.25;   // 25 cm step = non traversable
        TerrainAnalysisConfig analysis_config;


        vector<double> vector_roughness_lidar_raw;



        vector<double> vector_error;
        vector<double> vector_coordinates_local_x;
        vector<double> vector_coordinates_local_y;
        vector<double> vector_coordinates_global_x;
        vector<double> vector_coordinates_global_y;



        // Local and global initialization
        float local_map_size;
        float global_map_size;
        int nb_cells_local;
        int nb_cells_global;
        int offset_static = 0;
        float update_frequency;
        std::string map_frame_id = "map";
        std::string robot_frame_id = "base_footprint";
        bool debug_logging = false;
        bool publish_debug_maps = true;
        bool rebuild_global_map_on_cloud = true;
        bool compute_on_cloud_update = false;
        bool publish_on_timer = true;
        bool one_shot = false;
        bool allow_identity_pose_fallback = true;
        bool publish_global_on_update_only = true;
        bool publish_local_map = true;
        bool enable_footprint_inflation = false;
        bool cloud_received = false;
        bool map_dirty = false;
        bool has_robot_pose = false;
        int step_window_radius_cells = 1;
        int footprint_radius_cells = 1;
        int max_points_per_cell = 80;
        int cloud_point_stride = 1;
        double robot_radius = 0.45;
        double global_map_growth_margin = 5.0;
        double global_map_growth_step = 20.0;
        double global_map_max_size = 300.0;
        int floating_floor_neighbor_radius = 2;
        double max_floor_height_jump = 0.80;

        



        // ===========================
        // Methods
        // ===========================
        void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void publish_t_score_map(const TerrainGrid &grid, bool is_local);
        void publish_debug_maps_for_grid(const TerrainGrid &grid, bool is_local);

        void compute_t_score();


    protected:
        // ===========================
        // Attributes
        // ===========================
        // ROS 2
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_t_score_local_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_t_score_global_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_slope_global_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_roughness_global_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_height_global_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_confidence_global_;
        rclcpp::TimerBase::SharedPtr timer_tf_;


        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::TransformStamped transform_stamped;


        // Global map
        float resolution;           // Resolution of both local and global map


        // Rover coordinates
        coordinates robot_coordinates = {0,0,0};
        GridCoord robot_coordinates_grid;

        // Initialize PCL pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;        
        std::vector<GridCoord> occupied_global_cells;

        // General parameters
        rapidjson::Document p;      // Config file reader
        
        // ===========================
        // Methods
        // ===========================
        void get_parameters(std::string parameters_path);
        void lookupTransform();
        bool updateRobotPose();
        bool transformPointCloudToMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg, pcl::PointCloud<pcl::PointXYZ>& output);
        pcl::PointXYZ transformPoint(const pcl::PointXYZ& point, const geometry_msgs::msg::TransformStamped& transform) const;
        void shiftOccupiedCells(int shift_x, int shift_y);
        void expandGlobalGridForCloud(const pcl::PointCloud<pcl::PointXYZ>& input);
        void analyze_global_grid();
        size_t suppress_floating_floor_cells();
        void compute_step_heights_for_occupied();
        void update_local_grid_from_global();
        void apply_footprint_inflation(TerrainGrid& grid);
        nav_msgs::msg::OccupancyGrid make_occupancy_grid_message(const TerrainGrid& grid, bool is_local) const;
        nav_msgs::msg::OccupancyGrid make_metric_grid_message(const TerrainGrid& grid, bool is_local, const std::string& metric) const;
        void updateAndPublish();
};
