/**
 * @file ROSWrapper.cpp
 * @author XX.XX@uni.XX
 * @version 0.1
 * @date 2024-09-20
 * 
 * @copyright XX XX | 2024
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
    save_data = p["save_data"].GetBool();
    debug_info = p["debug_info"].GetBool();
    use_idw = p["activate_idw"].GetBool();
    imu_correction = p["activate_imu_correction"].GetBool();
    debug_time_s = p["debug_time_s"].GetInt();
    resolution = p["map_resolution"].GetFloat();
    roughness_lidar_threshold=p["roughness_lidar_threshold"].GetFloat();
    roughness_imu_threshold=p["roughness_imu_threshold"].GetFloat();
    window_size = p["imu_window_size"].GetInt();
    local_map_size = p["local_map_size"].GetFloat();
    global_map_size = p["global_map_size"].GetFloat();

    tf_frequency = p["tf_frequency"].GetFloat();
    tf_frequency_int = tf_frequency*1000;

    roughness_frequency = p["roughness_frequency"].GetFloat();
    roughness_frequency_int = roughness_frequency*1000;

    // Local and global intialization
    nb_cells_local = static_cast<unsigned int>(local_map_size/resolution);
    if ((nb_cells_local % 2) == 0)
    {
        nb_cells_local +=1;
    }
    nb_cells_global = static_cast<unsigned int>(global_map_size/resolution);
    if ((nb_cells_global % 2) == 0)
    {
        nb_cells_global +=1;
    }




    // =====================================================
    // Compute static offset (between global and local)
    // =====================================================
    int global_origin_index = nb_cells_global / 2;
    int local_origin_index = nb_cells_local / 2;
    offset_static = (global_origin_index - local_origin_index);

    // =====================================================
    // Initialize global map
    // =====================================================
    this->create_global_map();

    // =====================================================
    // TRANSFORM
    // =====================================================
    // Get transform of the robot
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_tf_ = this->create_wall_timer(
      std::chrono::milliseconds(tf_frequency_int),  // Call every time t (in ms)
      std::bind(&ROSWrapper::lookupTransform, this)
    );

    // timer_tf_ = rclcpp::create_timer(
    //     this->get_node_base_interface(),
    //     this->get_node_timers_interface(),
    //     this->get_clock(),
    //     std::chrono::milliseconds(tf_frequency_int),
    //     std::bind(&ROSWrapper::lookupTransform, this)
    // );



    timer_roughness_ = this->create_wall_timer(
      std::chrono::milliseconds(roughness_frequency_int),  // Call every time t (in ms)
      std::bind(&ROSWrapper::compute_roughness, this)
    );

    // timer_roughness_ = rclcpp::create_timer(
    //     this->get_node_base_interface(),
    //     this->get_node_timers_interface(),
    //     this->get_clock(),
    //     std::chrono::milliseconds(roughness_frequency_int),
    //     std::bind(&ROSWrapper::compute_roughness, this)
    // );



    // =====================================================
    // SPEED
    // =====================================================
    // Subscribe to the point cloud topic    
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
     p["cmd_vel_topic"].GetString(), 10, std::bind(&ROSWrapper::cmd_vel_callback, this, _1));
    


    // =====================================================
    // POINT CLOUD
    // =====================================================
    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Subscribe to the point cloud topic    
    sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
     p["pc_topic"].GetString(), 10, std::bind(&ROSWrapper::pc_callback, this, _1));
    

    // Create publishers
    pub_roughness_local_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(p["roughness_topic_local"].GetString(), 10);

    pub_roughness_global_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(p["roughness_topic_global"].GetString(), 10);


    // Set roughness parameters
    roughness.resolution = resolution;
    roughness.local_size = p["local_map_size"].GetFloat();
    roughness.global_size = p["global_map_size"].GetFloat();

    


    roughness.stylized=p["map_unknown_transparency"].GetBool();
    roughness.ransac_iterations=p["ransac_iterations"].GetInt();
    roughness.roughness_shift=p["roughness_shift"].GetFloat();
    roughness.roughness_lidar_threshold=p["roughness_lidar_threshold"].GetFloat();
    roughness.roughness_imu_threshold=p["roughness_imu_threshold"].GetFloat();
    roughness.height=p["height"].GetFloat();
    roughness.low_grid_resolution=p["map_low_resolution_division_factor"].GetInt();
    roughness.nb_cells = nb_cells_local;
    roughness.nb_cells_global = nb_cells_global;

    // =====================================================
    // IMU
    // =====================================================
    imu_sampling_frequency = p["imu_sampling_frequency"].GetFloat();
    imu_filter_frequency = p["imu_filter_frequency"].GetFloat();
    imu_filter_bandwidth = p["imu_filter_bandwidth"].GetFloat();
    DSP_ = std::make_shared<Dsp>();
    DSP_->create_filter(imu_filter_frequency, imu_filter_bandwidth, imu_sampling_frequency);

    // this->simulate_sinusoid_signal();    // Debug only
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
     p["imu_topic"].GetString(), 10, std::bind(&ROSWrapper::imu_callback, this, _1));


    // RLS and IDW initialization
    DSP_->lambda=p["rls_forgeting_factor"].GetFloat();
    DSP_->idw_power=p["idw_power"].GetFloat();
    DSP_->initialize_rls();

    RCLCPP_INFO(this->get_logger(), "Now: %.3f (sim time = %s)", 
    this->get_clock()->now().seconds(),
    this->get_parameter("use_sim_time").as_bool() ? "true" : "false");



};


ROSWrapper::~ROSWrapper() 
{
  if(save_data)
  {
    save_filtered_data();
    save_roughness_data();
    save_timers_data();
    // save_global_map();
  }
};




// =====================================================
// CALLBACKS
// =====================================================

void ROSWrapper::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    cloud->points.clear();
    pcl::fromROSMsg(*msg, *cloud);
};


void ROSWrapper::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double norm_linear = roughness.calculateNorm(msg->linear.x, msg->linear.y, msg->linear.z);
    double norm_angular = roughness.calculateNorm(msg->angular.x, msg->angular.y, msg->angular.z);
    velocity_norm = norm_linear + norm_angular;
};


void ROSWrapper::imu_callback(const sensor_msgs::msg::Imu &msg)
{
    
    // Calculate the norm of the linear acceleration vector
    double norm = roughness.calculateNorm(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    this->rawData.push_back(norm);

    // Initialize filtered variable
    complex<float> input(norm, 0.0f);
    complex<float> filtered_data;
      
    // Execute one filtering iteration.
    DSP_->process_sample(input, &filtered_data);

    this->filteredData.push_back(filtered_data.real());

    // Add observed data to the vector
    observed_data_filtered.push_back(filtered_data.real());
    this->update_window_imu(filtered_data.real());

    // Publish data (debug only)
    if (debug_info)
    {
      RCLCPP_INFO(this->get_logger(), "Norm: %.3f Filtered data: %.3f", norm, filtered_data.real());
    }
};



// =====================================================
// TRANSFORM
// =====================================================

void ROSWrapper::lookupTransform()
{
    try {

      rclcpp::Time now = this->get_clock()->now();
      rclcpp::Time safe_time = now - rclcpp::Duration::from_seconds(0.2);  // 50 ms safety margin


      // RCLCPP_INFO(this->get_logger(), "Lookup transform");
      
      // transform_stamped = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);

      // if (tf_buffer_->canTransform("map", "base_footprint", safe_time, rclcpp::Duration::from_seconds(0.1))) {
      //     transform_stamped = tf_buffer_->lookupTransform("map", "base_footprint", safe_time, rclcpp::Duration::from_seconds(0.1));
      //     RCLCPP_INFO(this->get_logger(), "Transform OK at %.3f", safe_time.seconds());
      // } else {
      //     RCLCPP_WARN(this->get_logger(), "Transform not available at %.3f", safe_time.seconds());
      // }
      
      // // Lookup transform from 'odom' to 'base_link'
      // rclcpp::Time now = this->get_clock()->now() - rclcpp::Duration::from_seconds(0.05);
      // transform_stamped = tf_buffer_->lookupTransform("map", "base_footprint", now, rclcpp::Duration::from_seconds(0.1));
      transform_stamped = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      
      roughness.pose = {transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z};
      coordinates_local = {roughness.pose[0], roughness.pose[1]};

      // RCLCPP_INFO(this->get_logger(), "Coordinates local: %.3f, %.3f", coordinates_local[0], coordinates_local[1]);


      new_cell = coord_local_to_global({nb_cells_local/2, nb_cells_local/2});
      // new_cell = compute_offset();

      if (new_cell != current_cell)
      {
        previous_cell = current_cell;
        current_cell = new_cell;
        changed_cell = true;
      }
      
    } 
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
      if ((temp_timer>=debug_time_s*5) && debug_info)
      {
        temp_timer = 0;
        RCLCPP_WARN(this->get_logger(), "Could not transform 'odom' to 'base_link': %s", ex.what());
        RCLCPP_WARN(this->get_logger(), "Is the TF topic properly published|filled ?");
      }
      else
      {
        temp_timer +=1;
      }
    }
};




void ROSWrapper::compute_roughness()
{

  if (previous_cell[0] == 0 || previous_cell[1] == 0)
    {
      RCLCPP_INFO(this->get_logger(), "No previous cell, skipping roughness computation");
      return;
    }
  if(!cloud->empty() && changed_cell)
  {
      RCLCPP_INFO(this->get_logger(), "Compute roughness");

      timer_struct timer_chrono;

      rclcpp::Time start = this->get_clock()->now();
      roughness.CalculatePCRoughness(cloud);
      rclcpp::Time end = this->get_clock()->now();
      long long duration = (end - start).nanoseconds();
      timer_chrono.pc_roughness_timer = duration;
      duration = 0;

      start = this->get_clock()->now();
      // roughness.saveEntireGridToPCD(roughness.PCGrid);
      end = this->get_clock()->now();
      duration = (end - start).nanoseconds();
      timer_chrono.save_entire_pc_timer = duration;
      duration = 0;



      start = this->get_clock()->now();
      coordinates_grid local_coordinates = coord_global_to_local(previous_cell);
      end = this->get_clock()->now();
      duration = (end - start).nanoseconds();
      timer_chrono.calculating_coordonates_timer = duration;
      duration = 0;

      // Calculate the IMU roughness
      start = this->get_clock()->now();
      vector<double> observed_values_speed_normalized = speed_normalization(observed_data_filtered);
      end = this->get_clock()->now();
      duration = (end - start).nanoseconds();
      timer_chrono.speed_normalization_timer = duration;
      duration = 0;


      start = this->get_clock()->now();
      roughness.CalculateObservedRoughness(observed_values_speed_normalized, local_coordinates);
      observed_data_filtered.clear();
      end = this->get_clock()->now();
      duration = (end - start).nanoseconds();
      timer_chrono.calculate_observed_roughness_timer = duration;
      duration = 0;
      
      
      start = this->get_clock()->now();
      // Both roughness values and IMU values are available
      if (!std::isfinite(roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][1]) || !std::isfinite(roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][3])) 
      {
        RCLCPP_WARN(this->get_logger(), "Non-finite roughness values, skipping RLS update");
        return;
      }
      DSP_->rls_update(roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][1], roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][3]);
      roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][4] =  DSP_->theta[0];
      roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][5] =  DSP_->theta[1];
      end = this->get_clock()->now();
      duration = (end - start).nanoseconds();
      timer_chrono.rls_update_timer = duration;
      duration = 0;

      // RLS
      rclcpp::Time start_rls = this->get_clock()->now();

      // for (int i = 0; i < roughness.TGridLocal.size(); i++)
      // {
      //   for (int j = 0; j < roughness.TGridLocal.size(); j++)
      //   {
      //     if (roughness.TGridLocal[i][j][1] == -1)
      //     {
      //       continue;
      //     }
      //     else if (roughness.TGridLocal[i][j][1] == 0)
      //     {
      //       continue;
      //     }
      //     else
      //     {
      //       double roughness_corrected = DSP_->rls_correction(roughness.TGridLocal[i][j][1]);
      //       roughness.TGridLocal[i][j][6] = roughness.roughness_normalization(roughness_corrected, 1);
      //     }
          
      //   }
      // }

      auto is_plausible = [&](double v)
      {
        return std::isfinite(v) && v >= rough_min && v <= rough_max;
      };

      for (int i = 0; i < roughness.TGridLocal.size(); ++i) 
      {
          // if rows can differ, prefer roughness.TGridLocal[i].size()
          for (int j = 0; j < roughness.TGridLocal[i].size(); ++j) {
              double rpc = roughness.TGridLocal[i][j][1];

              // Treat -1 or non-finite as "missing"
              if (!std::isfinite(rpc) || rpc == -1.0) {
                  continue; // no correction possible
              }

              // If 0 is sometimes a "no data" sentinel, treat it as missing:
              // if (rpc == 0.0) continue;

              if (!is_plausible(rpc)) {
                  continue; // drop crazy values
              }

              double corrected = DSP_->rls_correction(rpc);
              if (!std::isfinite(corrected)) {
                  continue; // avoid writing NaNs back to the grid
              }

              roughness.TGridLocal[i][j][6] = roughness.roughness_normalization(corrected, 1);
          }
      }
      rclcpp::Time end_rls = this->get_clock()->now();
      duration = (end_rls - start_rls).nanoseconds();
      timer_chrono.rls_correction_timer = duration;
      duration = 0;

      

      Cell current_cell_struct;
      current_cell_struct.local_x = local_coordinates[0];
      current_cell_struct.local_y = local_coordinates[1];
      current_cell_struct.global_x = previous_cell[0];
      current_cell_struct.global_y = previous_cell[1];

      current_cell_struct.pred_raw = roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][0];
      current_cell_struct.pred_norm = roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][1];
      current_cell_struct.observed_raw = roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][2];
      current_cell_struct.observed_norm = roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][3];
      current_cell_struct.theta_a = roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][4];
      current_cell_struct.theta_b = roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][5];
      current_cell_struct.roughness_corrected = roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][6];


      start = this->get_clock()->now();
      // IDW
      if (use_idw)
      {
        for (int i = 0; i < roughness.TGridLocal.size(); i++)
        {
          for (int j = 0; j < roughness.TGridLocal.size(); j++)
          {
            roughness.TGridLocal[i][j][8] = DSP_->idw_interpolation(vector_observed_cells, i, j, roughness.TGridLocal[i][j][1]);
          }
        }
      }
      else
      {
        for (int i = 0; i < roughness.TGridLocal.size(); i++)
        {
          for (int j = 0; j < roughness.TGridLocal.size(); j++)
          {
            roughness.TGridLocal[i][j][8] = roughness.TGridLocal[i][j][6];
          }
        }
      }
      end = this->get_clock()->now();
      duration = (end - start).nanoseconds();
      timer_chrono.idw_interpolation_timer = duration;
      duration = 0;

      current_cell_struct.delta_idw = roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][7];
      current_cell_struct.roughness_final = roughness.TGridLocal[local_coordinates[0]][local_coordinates[1]][8];



      start = this->get_clock()->now();
      update_global_map(current_cell);
      end = this->get_clock()->now();
      duration = (end - start).nanoseconds();
      timer_chrono.update_global_map_timer = duration;
      duration = 0;


      // Publish maps
      start = this->get_clock()->now();
      publish_roughness_map(roughness.TGridLocal, true);  // Local map
      publish_roughness_map(global_grid, false);          // Global map
      end = this->get_clock()->now();
      duration = (end - start).nanoseconds();
      timer_chrono.publish_maps_timer = duration;

      changed_cell = false;
      vector_observed_cells.push_back(current_cell_struct);
      vector_times.push_back(timer_chrono);

      save_global_map();
      
      // vector_global_maps.push_back(global_grid);
  }
};




// =====================================================
// PUBLISHER
// =====================================================

void ROSWrapper::publish_roughness_map(const TerrainGrid &grid, bool is_local)
{
    RCLCPP_INFO(this->get_logger(), "Publish map");

    nav_msgs::msg::OccupancyGrid occupancy_grid;
    nav_msgs::msg::MapMetaData map_meta_data;
    map_meta_data.map_load_time = this->now();
    map_meta_data.resolution = resolution;
    map_meta_data.width =  grid.size();
    map_meta_data.height = grid[0].size();

    float center_coordinates_x = (-(grid.size()*resolution)/2);
    float center_coordinates_y = (-(grid[0].size()*resolution)/2);


    if (is_local)
    {
      map_meta_data.origin.position.x = center_coordinates_x + coordinates_local[0] + resolution/2;
      map_meta_data.origin.position.y = center_coordinates_y + coordinates_local[1] + resolution/2;
    }
    else
    {
      map_meta_data.origin.position.x = center_coordinates_x + resolution/2;
      map_meta_data.origin.position.y = center_coordinates_y + resolution/2;
    }
    map_meta_data.origin.position.z = transform_stamped.transform.translation.z;

    map_meta_data.origin.orientation.x= 0.0;
    map_meta_data.origin.orientation.y= 0.0;


    map_meta_data.origin.orientation.z= 0.0;
    map_meta_data.origin.orientation.w= 1.0;

    
    occupancy_grid.header.stamp = this->now();
    occupancy_grid.header.frame_id = p["roughness_frame_id"].GetString();
    occupancy_grid.info = map_meta_data;
    occupancy_grid.data.resize(map_meta_data.width * map_meta_data.height, -1);

    // Fill the occupancy_grid with data
    // Fill the OccupancyGrid message.
    for (unsigned int y = 0; y < map_meta_data.height; ++y) 
    {
      for (unsigned int x = 0; x < map_meta_data.width; ++x) 
      {
        unsigned int index = y * map_meta_data.width + x;
        occupancy_grid.data[index] = grid[x][y][1]*100;                    // Fill data
      }
    }

    if(is_local)
    {
      pub_roughness_local_->publish(occupancy_grid);
    }
    else
    {
      pub_roughness_global_->publish(occupancy_grid);
    }
};




// =====================================================
// TOOLS
// =====================================================

// Generate sinusoid signal for IMU debug 
void ROSWrapper::simulate_sinusoid_signal()
{
  RCLCPP_INFO(this->get_logger(), "Generating simulated data");
  DSP_->generate_simulated_signal();
  RCLCPP_INFO(this->get_logger(), "Size of generated data: %.3ld", DSP_->sinusoid.size());
  for(int i=0; i<DSP_->sinusoid.size(); i++)
  {
    complex<float> input(DSP_->sinusoid[i], 0.0f);
    complex<float> output;

    DSP_->process_sample(input, &output);

    // double filtered_data = DSP_->processSample(DSP_->sinusoid[i]);
    this->filteredData.push_back(output.real());
    RCLCPP_INFO(this->get_logger(), "Data: %.3f Filtered data: %.3f", DSP_->sinusoid[i], output.real());
  }

  // Open an output file stream (CSV file).
  std::ofstream outFile("data.csv");
  if (!outFile.is_open()) 
  {
      std::cerr << "Error opening file for writing!" << std::endl;
  }

  // Write a header row (optional).
  outFile << "Index,Raw,Filtered\n";

  // Write data row by row.
  for (size_t i = 0; i < DSP_->sinusoid.size(); i++) 
  {
      outFile << i << "," << DSP_->sinusoid[i] << "," << this->filteredData[i] << "\n";
  }

  outFile.close();
  std::cout << "Data exported to data.csv" << std::endl;
};


// Update the sliding window with new IMU data 
void ROSWrapper::update_window_imu(double x)
{
    if (window_imu.size() == window_size) {
            window_imu.pop_front();  // Remove the oldest value
        }
        window_imu.push_back(x);
};


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


// Save the filtered data
void ROSWrapper::save_filtered_data()
{
  // Open an output file stream (CSV file).
  std::ofstream outFile("data/data_imu_filtering.csv");
  if (!outFile.is_open()) 
  {
      RCLCPP_ERROR(this->get_logger(), "Error opening file for writing.");
      return;
  }

  // Write a header row (optional).
  outFile << "Index,Raw,Filtered\n";

  // Write data row by row.
  for (size_t i = 0; i < this->rawData.size(); i++) 
  {
      outFile << i << "," << this->rawData[i] << "," << this->filteredData[i] << "\n";
  }

  outFile.close();
  RCLCPP_INFO(this->get_logger(), "Filtered data saved to data_imu_filtering.csv");
};


// Save IMU and LiDAR roughness for analysis
void ROSWrapper::save_roughness_data()
{
  // Open an output file stream (CSV file).
  std::ofstream outFile("data/sensors/data_analysis.csv");
  if (!outFile.is_open()) 
  {
      RCLCPP_ERROR(this->get_logger(), "Error opening file for writing.");
      return;
  }

  // Write a header row (optional).
  outFile << "Index,local_x,local_y,global_x,global_y,pred_raw,pred_norm,observed_raw,observed_norm,theta_a,theta_b,roughness_corrected,delta_idw,roughness_final\n";

  // Write data row by row.
  for (size_t i = 0; i < this->vector_observed_cells.size(); i++) 
  {
      outFile << i << "," << 
      this->vector_observed_cells[i].local_x << "," << 
      this->vector_observed_cells[i].local_y << "," << 
      this->vector_observed_cells[i].global_x << "," << 
      this->vector_observed_cells[i].global_y << "," << 
      this->vector_observed_cells[i].pred_raw << "," << 
      this->vector_observed_cells[i].pred_norm << "," <<
      this->vector_observed_cells[i].observed_raw << "," <<
      this->vector_observed_cells[i].observed_norm << "," <<
      this->vector_observed_cells[i].theta_a << "," <<
      this->vector_observed_cells[i].theta_b << "," <<
      this->vector_observed_cells[i].roughness_corrected << "," <<
      this->vector_observed_cells[i].delta_idw << "," <<
      this->vector_observed_cells[i].roughness_final
      << "\n";
  }

  outFile.close();
  RCLCPP_INFO(this->get_logger(), "Experiment data saved to data_analysis.csv");
};



void ROSWrapper::save_timers_data()
{
  // Open an output file stream (CSV file).
  std::ofstream outFile("data/sensors/timer_analysis.csv");
  if (!outFile.is_open()) 
  {
      RCLCPP_ERROR(this->get_logger(), "Error opening file for writing.");
      return;
  }

  // Write a header row (optional).
  outFile << "Index,pc_roughness_timer,save_entire_pc_timer,calculating_coordonates_timer,speed_normalization_timer,calculate_observed_roughness_timer,rls_update_timer,rls_correction_timer,idw_interpolation_timer,update_global_map_timer,publish_maps_timer\n";

  // Write data row by row.
  for (size_t i = 0; i < this->vector_times.size(); i++) 
  {
      outFile << i << "," << 
      this->vector_times[i].pc_roughness_timer << "," <<
      this->vector_times[i].save_entire_pc_timer << "," <<
      this->vector_times[i].calculating_coordonates_timer << "," <<
      this->vector_times[i].speed_normalization_timer << "," <<
      this->vector_times[i].calculate_observed_roughness_timer << "," <<
      this->vector_times[i].rls_update_timer << "," <<
      this->vector_times[i].rls_correction_timer << "," <<
      this->vector_times[i].idw_interpolation_timer << "," <<
      this->vector_times[i].update_global_map_timer << "," <<
      this->vector_times[i].publish_maps_timer
      << "\n";
  }

  outFile.close();
  RCLCPP_INFO(this->get_logger(), "Experiment data saved to timer_analysis.csv");
};



void ROSWrapper::save_global_map()
{
  string filename_root = "data/global_maps/global_map_";

  string filename = filename_root + to_string(id_global_map) + ".csv";
  ofstream file(filename);
  
  if (!file.is_open()) {
      cerr << "Error: Could not open file " << filename << endl;
      return;
  }

  int rows = global_grid.size();
  int cols = global_grid[0].size();
  int layers = global_grid[0][0].size();


  // Write header
  file << "Row,Col";
  for (int l = 0; l < layers; ++l) {
      file << ",Layer_" << l;
  }
  file << "\n";

  // Write global_grid data
  for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        if ((global_grid[r][c][0]==0) && (global_grid[r][c][1]==0) && (global_grid[r][c][2]==0) && (global_grid[r][c][3]==0) && (global_grid[r][c][4]==0) && (global_grid[r][c][5]==0) && (global_grid[r][c][6]==0) && (global_grid[r][c][7]==0) && (global_grid[r][c][8]==0))
              {
                continue;
              }
        else
        {
          file << r << "," << c;  // Row and Column indices
          for (int l = 0; l < layers; ++l) {
              file << "," << global_grid[r][c][l];  // Values from all layers
          }
          file << "\n";
        }
      }
  }

  file.close();
  RCLCPP_INFO(this->get_logger(), "Grid saved as %s", filename.c_str());
  id_global_map +=1;
};


// Convert a deque of double into a vector of double
vector<double> ROSWrapper::convert_deque_vector(deque<double> input)
{
  // Create a vector of floats with the same size as imuData
  vector<double> output(input.size());

  // Convert each double to double using transform
  transform(input.begin(), input.end(), output.begin(),
                  [](double d) { return static_cast<double>(d); });
  
  return output;
};


// Create global map
void ROSWrapper::create_global_map()
{
  global_grid.clear();
  global_grid.resize(nb_cells_global);
  for(int ind_x=0; ind_x<nb_cells_global; ind_x++)
  {
      global_grid[ind_x].resize(nb_cells_global);
      for(int ind_y=0; ind_y<nb_cells_global; ind_y++)
      {
          global_grid[ind_x][ind_y].resize(9);
          global_grid[ind_x][ind_y][0] = 0.0;
          global_grid[ind_x][ind_y][1] = 0.0;
          global_grid[ind_x][ind_y][2] = 0.0;
          global_grid[ind_x][ind_y][3] = 0.0;
          global_grid[ind_x][ind_y][4] = 0.0;
          global_grid[ind_x][ind_y][5] = 0.0;
          global_grid[ind_x][ind_y][6] = 0.0;
          global_grid[ind_x][ind_y][7] = 0.0;
          global_grid[ind_x][ind_y][8] = 0.0;
      }
  }
};


// Update global map
void ROSWrapper::update_global_map(coordinates_grid offset)
{
  // Loop on each cell of the local grid
  for(int local_ind_x=0; local_ind_x<roughness.TGridLocal.size(); local_ind_x++)
  {
    for(int local_ind_y=0; local_ind_y<roughness.TGridLocal[0].size(); local_ind_y++)
    {


      coordinates_grid global_coord = coord_local_to_global({local_ind_x, local_ind_y});

      // Check if the computed global cell is within the bounds of the global grid.&
      if (global_coord[0] < 0 || global_coord[0] >= static_cast<int>(global_grid.size()) ||
          global_coord[1] < 0 || global_coord[1] >= static_cast<int>(global_grid.size()))
      {
        // Skip cells that fall outside the global grid.
        RCLCPP_ERROR(this->get_logger(), "Error: Local grid outside the global grid.");
        continue;
      }

      if (roughness.TGridLocal[local_ind_x][local_ind_y][0]>0) // Only takes the computed values
      {
        for (int i=0; i<9; i++)
        {
          if ((i!=2) && (i!=3) && (i!=4) && (i!=5) && (i!=7))
          {
            global_grid[global_coord[0]][global_coord[1]][i] = roughness.TGridLocal[local_ind_x][local_ind_y][i];
          }
          else
          {
            coordinates_grid index_pose = coord_local_to_global({local_ind_x, local_ind_y});
            if((index_pose[0] == previous_cell[0]) && (index_pose[1] == previous_cell[1]))
            {
              global_grid[previous_cell[0]][previous_cell[1]][i] = roughness.TGridLocal[local_ind_x][local_ind_y][i];
            }
          }
        }
          

          //Save current cell
          // roughness.SaveCellASPC(local_ind_x, local_ind_y, vector_roughness_lidar_raw.size());
        // }
      }

      // global_grid[global_coord[0]][global_coord[1]] = roughness.TGridLocal[local_ind_x][local_ind_y];
          //Save current cell
          // roughness.SaveCellASPC(local_ind_x, local_ind_y, vector_roughness_lidar_raw.size());
    }
  }
};


// Divide all the Norms by the speed to remove the speed variation
vector<double> ROSWrapper::speed_normalization(vector<double>& norms)
{
  if (velocity_norm != 0.0)
  {
    if(velocity_norm < 1.0){velocity_norm*=10;};
    for (int i=0; i<norms.size(); i++)
    {
      norms[i] /=  velocity_norm;
    }
  }
  return norms;
};


coordinates_grid ROSWrapper::coord_local_to_global(coordinates_grid coord)
{
  int dynamic_offset_x = 0;
  int dynamic_offset_y = 0;

  if (roughness.pose[0] != 0)
  {
    dynamic_offset_x = roughness.pose[0] / resolution;
  }
  if (roughness.pose[1] != 0)
  {
    dynamic_offset_y = roughness.pose[1] / resolution;
  }
  
  int global_cell_x =  coord[0] + offset_static + dynamic_offset_x; // local coordinate + offset + dynamic offset
  int global_cell_y =  coord[1] + offset_static + dynamic_offset_y; // local coordinate + offset + dynamic offset

  // Ensure local coordinates remain within valid bounds
  global_cell_x = max(0, min(global_cell_x, nb_cells_global - 1));
  global_cell_y = max(0, min(global_cell_y, nb_cells_global - 1));

  return {global_cell_x, global_cell_y};
};

coordinates_grid ROSWrapper::coord_global_to_local(coordinates_grid coord)
{
  int dynamic_offset_x = 0;
  int dynamic_offset_y = 0;

  if (roughness.pose[0] != 0)
  {
    dynamic_offset_x = roughness.pose[0] / resolution;
  }
  if (roughness.pose[1] != 0)
  {
    dynamic_offset_y = roughness.pose[1] / resolution;
  }
  int local_cell_x  =  coord[0] - offset_static - dynamic_offset_x; // global coordinate - offset - dynamic offset
  int local_cell_y  =  coord[1] - offset_static - dynamic_offset_y; // global coordinate - offset - dynamic offset

  // Ensure local coordinates remain within valid bounds
  local_cell_x = max(0, min(local_cell_x, nb_cells_local - 1));
  local_cell_y = max(0, min(local_cell_y, nb_cells_local - 1));


  return {local_cell_x, local_cell_y};
};


// =====================================================
// MAIN
// =====================================================


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSWrapper>());
  rclcpp::shutdown();
  return 0;
}
