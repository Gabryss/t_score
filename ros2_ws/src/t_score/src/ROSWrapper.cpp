/**
 * @file ROSWrapper.cpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2025-11-19
 * 
 * @copyright Gabriel Garcia | 2025
 * @brief Implementation file for Class ROSWrapper.
 * @details This is a ROS wrapper that subscribe to a pointcloud and publish a roughness costmap.
 */
#include "ROSWrapper.hpp"



ROSWrapper::ROSWrapper(): Node("roughness_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    // =====================================================
    // GET CONFIG PARAMETERS
    // =====================================================
    this->declare_parameter("param_path", "");
    std::string path_parameters = this->get_parameter("param_path").as_string();
    get_parameters(path_parameters);

    resolution = p["map_resolution"].GetFloat();
    local_map_size = p["local_map_size"].GetFloat();
    global_map_size = p["global_map_size"].GetFloat();

    tf_frequency = p["tf_frequency"].GetFloat();
    tf_frequency_int = tf_frequency*1000;


    // Initialize grid
    grid_manager.create_local_global_grids(global_map_size, local_map_size, resolution);
    std::cerr << "Global grid size: " << grid_manager.global_grid.size()<< std::endl;
    std::cerr << "Global grid size: " << grid_manager.local_grid.size()<< std::endl;



    // =====================================================
    // Compute static offset (between global and local)
    // =====================================================
    // int global_origin_index = nb_cells_global / 2;
    // int local_origin_index = nb_cells_local / 2;
    // offset_static = (global_origin_index - local_origin_index);

    // =====================================================
    // Initialize global map
    // =====================================================
    // this->create_global_map();

    // =====================================================
    // TRANSFORM
    // =====================================================
    // Get transform of the robot
    // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // timer_tf_ = this->create_wall_timer(
    //   std::chrono::milliseconds(tf_frequency_int),  // Call every time t (in ms)
    //   std::bind(&ROSWrapper::lookupTransform, this)
    // );

    // timer_tf_ = rclcpp::create_timer(
    //     this->get_node_base_interface(),
    //     this->get_node_timers_interface(),
    //     this->get_clock(),
    //     std::chrono::milliseconds(tf_frequency_int),
    //     std::bind(&ROSWrapper::lookupTransform, this)
    // );





    // =====================================================
    // POINT CLOUD
    // =====================================================
    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Subscribe to the point cloud topic    
    sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
     p["pc_topic"].GetString(), 10, std::bind(&ROSWrapper::pc_callback, this, _1));
    

    // Create publishers
    pub_roughness_local_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(p["traversability_topic_local"].GetString(), 10);

    pub_roughness_global_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(p["traversability_topic_global"].GetString(), 10);






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
    cloud->points.clear();
    pcl::fromROSMsg(*msg, *cloud);
};



// =====================================================
// TRANSFORM
// =====================================================


// void ROSWrapper::lookupTransform()
// {
//     try {

//       rclcpp::Time now = this->get_clock()->now();
//       rclcpp::Time safe_time = now - rclcpp::Duration::from_seconds(0.2);  // 50 ms safety margin


//       // RCLCPP_INFO(this->get_logger(), "Lookup transform");
      
//       // transform_stamped = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);

//       // if (tf_buffer_->canTransform("map", "base_footprint", safe_time, rclcpp::Duration::from_seconds(0.1))) {
//       //     transform_stamped = tf_buffer_->lookupTransform("map", "base_footprint", safe_time, rclcpp::Duration::from_seconds(0.1));
//       //     RCLCPP_INFO(this->get_logger(), "Transform OK at %.3f", safe_time.seconds());
//       // } else {
//       //     RCLCPP_WARN(this->get_logger(), "Transform not available at %.3f", safe_time.seconds());
//       // }
      
//       // // Lookup transform from 'odom' to 'base_link'
//       // rclcpp::Time now = this->get_clock()->now() - rclcpp::Duration::from_seconds(0.05);
//       // transform_stamped = tf_buffer_->lookupTransform("map", "base_footprint", now, rclcpp::Duration::from_seconds(0.1));
//       transform_stamped = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      
//       roughness.pose = {transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z};
//       coordinates_local = {roughness.pose[0], roughness.pose[1]};

//       // RCLCPP_INFO(this->get_logger(), "Coordinates local: %.3f, %.3f", coordinates_local[0], coordinates_local[1]);


//       new_cell = coord_local_to_global({nb_cells_local/2, nb_cells_local/2});
//       // new_cell = compute_offset();

//       if (new_cell != current_cell)
//       {
//         previous_cell = current_cell;
//         current_cell = new_cell;
//         changed_cell = true;
//       }
      
//     } 
//     catch (const tf2::TransformException & ex) {
//       RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
//       if ((temp_timer>=debug_time_s*5) && debug_info)
//       {
//         temp_timer = 0;
//         RCLCPP_WARN(this->get_logger(), "Could not transform 'odom' to 'base_link': %s", ex.what());
//         RCLCPP_WARN(this->get_logger(), "Is the TF topic properly published|filled ?");
//       }
//       else
//       {
//         temp_timer +=1;
//       }
//     }
// };





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





// coordinates_grid ROSWrapper::coord_local_to_global(coordinates_grid coord)
// {
//   int dynamic_offset_x = 0;
//   int dynamic_offset_y = 0;

//   if (roughness.pose[0] != 0)
//   {
//     dynamic_offset_x = roughness.pose[0] / resolution;
//   }
//   if (roughness.pose[1] != 0)
//   {
//     dynamic_offset_y = roughness.pose[1] / resolution;
//   }
  
//   int global_cell_x =  coord[0] + offset_static + dynamic_offset_x; // local coordinate + offset + dynamic offset
//   int global_cell_y =  coord[1] + offset_static + dynamic_offset_y; // local coordinate + offset + dynamic offset

//   // Ensure local coordinates remain within valid bounds
//   global_cell_x = max(0, min(global_cell_x, nb_cells_global - 1));
//   global_cell_y = max(0, min(global_cell_y, nb_cells_global - 1));

//   return {global_cell_x, global_cell_y};
// };

// coordinates_grid ROSWrapper::coord_global_to_local(coordinates_grid coord)
// {
//   int dynamic_offset_x = 0;
//   int dynamic_offset_y = 0;

//   if (roughness.pose[0] != 0)
//   {
//     dynamic_offset_x = roughness.pose[0] / resolution;
//   }
//   if (roughness.pose[1] != 0)
//   {
//     dynamic_offset_y = roughness.pose[1] / resolution;
//   }
//   int local_cell_x  =  coord[0] - offset_static - dynamic_offset_x; // global coordinate - offset - dynamic offset
//   int local_cell_y  =  coord[1] - offset_static - dynamic_offset_y; // global coordinate - offset - dynamic offset

//   // Ensure local coordinates remain within valid bounds
//   local_cell_x = max(0, min(local_cell_x, nb_cells_local - 1));
//   local_cell_y = max(0, min(local_cell_y, nb_cells_local - 1));


//   return {local_cell_x, local_cell_y};
// };




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSWrapper>());
  rclcpp::shutdown();
  return 0;
}