/**
 * @file ROSWrapper.hpp
 * @author XX.XX@uni.xx
 * @version 0.1
 * @date 2024-09-20
 * 
 * @copyright XX XX | 2024
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


// OPENCV
#include <opencv2/opencv.hpp>

// Custom library
#include "Roughness.hpp"
#include "Dsp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace cv;
using namespace cv::ml;

// Each cell is a vector<double> of size 2 representing terrain state.
using TerrainCell = vector<double>;  
// The global grid is a 2D grid where each cell is a TerrainCell.
using TerrainGrid = vector<vector<TerrainCell>>;

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
        vector<double> filteredData;
        vector<double> rawData;
        vector<double> observed_data_filtered;

        
        vector<double> vector_roughness_lidar_raw;
        vector<double> vector_roughness_lidar_raw_normalized;
        vector<double> vector_roughness_lidar_raw_normalized_corrected;


        vector<double> vector_roughness_imu;
        vector<double> vector_roughness_imu_normalized;

        vector<double> vector_velocity_imu;

        vector<double> vector_error;
        vector<double> vector_coordinates_local_x;
        vector<double> vector_coordinates_local_y;
        vector<double> vector_coordinates_global_x;
        vector<double> vector_coordinates_global_y;


        vector<Cell> vector_observed_cells;
        vector<timer_struct> vector_times;
        vector<TerrainGrid> vector_global_maps;


        

        float roughness_lidar_threshold=1;
        float roughness_imu_threshold=1;

        // Local and global initialization
        float local_map_size;
        float global_map_size;
        int nb_cells_local;
        int nb_cells_global;
        int offset_static = 0;
        float roughness_frequency;
        float tf_frequency;
        int roughness_frequency_int;
        int tf_frequency_int;


        // ===========================
        // Methods
        // ===========================
        void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu &msg);
        void publish_roughness_map(const TerrainGrid &grid, bool is_local);
        void simulate_sinusoid_signal();
        void save_filtered_data();
        void save_roughness_data();
        void save_timers_data();
        void save_global_map();


        void compute_roughness();

        




    protected:
        // ===========================
        // Attributes
        // ===========================
        // ROS 2
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_roughness_local_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_roughness_global_;
        rclcpp::TimerBase::SharedPtr timer_tf_;
        rclcpp::TimerBase::SharedPtr timer_roughness_;


        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::TransformStamped transform_stamped;

        // Roughness
        Roughness roughness;
        vector<double> observed_roughness_vector;
        double last_imu_roughness;

        // Global map
        TerrainGrid global_grid;
        float resolution;           // Resolution of both local and global map
        coordinates_grid new_cell = {0,0};
        coordinates_grid current_cell = {0,0};
        coordinates_grid previous_cell = {0,0};
        coordinates_grid local_grid_origin = {0,0};
        bool changed_cell = false;


        // Local map
        coordinates coordinates_local = {0,0};
        vector<long unsigned int> robot_imu_coordinates;

        // Notch bandstop filter;
        std::shared_ptr<Dsp> DSP_;
        float imu_filter_bandwidth;
        float imu_filter_frequency;
        float imu_sampling_frequency;

        // Initialize PCL pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;        

        // General parameters
        rapidjson::Document p;      // Config file reader
        bool debug_info;    
        bool save_data;
        int window_size = 3;        // Amount of stored data in the window
        deque<double> window_imu;   // IMU window
        int debug_time_s;           // In seconds
        int temp_timer = 0;
        double velocity_norm=0.3;       // Speed of the rover (default 0.3)
        float LETHAL = 100.0;
        bool imu_correction;
        bool use_idw;
        int id_global_map = 0;
        

        const double rough_min = 0.0;     // set to your plausible min
        const double rough_max = 0.5;     // set to your plausible max

        // ===========================
        // Methods
        // ===========================
        void get_parameters(std::string parameters_path);
        void lookupTransform();
        void update_window_imu(double x);
        void create_global_map();
        void update_global_map(coordinates_grid offset);
        vector<double> convert_deque_vector(deque<double> input);
        vector<double> speed_normalization(vector<double>& norms);
        coordinates_grid coord_local_to_global(coordinates_grid coord);
        coordinates_grid coord_global_to_local(coordinates_grid coord);


};
