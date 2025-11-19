/**
 * @file ROSWrapper.hpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2025-11-19
 * 
 * @copyright Gabriel Garcia | 2025
 * @brief Implementation file for Class ROSWrapper.
 * @details This is a ROS wrapper that subscribe to a pointcloud and publish a roughness costmap.
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

// Used to monitor time
struct timer_struct {
    long long pc_roughness_timer, save_entire_pc_timer, calculating_coordonates_timer, speed_normalization_timer, calculate_observed_roughness_timer, rls_update_timer, rls_correction_timer, idw_interpolation_timer, update_global_map_timer, publish_maps_timer;
};


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


        vector<double> vector_roughness_lidar_raw;
        vector<double> vector_roughness_lidar_raw_normalized;
        vector<double> vector_roughness_lidar_raw_normalized_corrected;



        vector<double> vector_error;
        vector<double> vector_coordinates_local_x;
        vector<double> vector_coordinates_local_y;
        vector<double> vector_coordinates_global_x;
        vector<double> vector_coordinates_global_y;


        // vector<Cell> vector_observed_cells;
        vector<timer_struct> vector_times;
        vector<TerrainGrid> vector_global_maps;


        // Local and global initialization
        float local_map_size;
        float global_map_size;
        int nb_cells_local;
        int nb_cells_global;
        int offset_static = 0;
        float tf_frequency;
        int tf_frequency_int;


        // ===========================
        // Methods
        // ===========================
        void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void publish_roughness_map(const TerrainGrid &grid, bool is_local);

        void compute_roughness();


    protected:
        // ===========================
        // Attributes
        // ===========================
        // ROS 2
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_roughness_local_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_roughness_global_;
        rclcpp::TimerBase::SharedPtr timer_tf_;
        rclcpp::TimerBase::SharedPtr timer_roughness_;


        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::TransformStamped transform_stamped;


        // Global map
        float resolution;           // Resolution of both local and global map
        coordinates_grid new_cell = {0,0};
        coordinates_grid current_cell = {0,0};
        coordinates_grid previous_cell = {0,0};
        coordinates_grid local_grid_origin = {0,0};
        bool changed_cell = false;


        // Local map
        coordinates coordinates_local = {0,0};


        // Initialize PCL pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;        

        // General parameters
        rapidjson::Document p;      // Config file reader
        bool debug_info;    
        
        // ===========================
        // Methods
        // ===========================
        void get_parameters(std::string parameters_path);
        // void lookupTransform();
        // void update_window_imu(double x);
        // void create_global_map();
        // void update_global_map(coordinates_grid offset);
        // vector<double> convert_deque_vector(deque<double> input);
        // vector<double> speed_normalization(vector<double>& norms);
        // coordinates_grid coord_local_to_global(coordinates_grid coord);
        // coordinates_grid coord_global_to_local(coordinates_grid coord);


};
