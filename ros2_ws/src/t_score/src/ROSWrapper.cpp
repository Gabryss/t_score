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



ROSWrapper::ROSWrapper(): Node("t_score_node", rclcpp::NodeOptions().use_intra_process_comms(true))
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

    update_frequency = p["update_frequency"].GetFloat();
    int update_frequency_int = static_cast<int>(update_frequency*1000); // Convert to ms


    // Initialize grid
    grid_manager.create_local_global_grids(global_map_size, local_map_size, resolution);
    std::cerr << "Global grid size: " << grid_manager.global_grid.size()<< std::endl;
    std::cerr << "Global grid size: " << grid_manager.local_grid.size()<< std::endl;



    

    // =====================================================
    // TRANSFORM
    // =====================================================
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_tf_ = this->create_wall_timer(
      std::chrono::milliseconds(update_frequency_int),  // Call every time t (in ms)
      std::bind(&ROSWrapper::lookupTransform, this)
    );

    timer_tf_ = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        this->get_clock(),
        std::chrono::milliseconds(update_frequency_int),
        std::bind(&ROSWrapper::lookupTransform, this)
    );





    // =====================================================
    // POINT CLOUD
    // =====================================================
    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Subscribe to the point cloud topic    
    sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
     p["pc_topic"].GetString(), 10, std::bind(&ROSWrapper::pc_callback, this, _1));
    

    // Create publishers
    pub_t_score_local_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(p["traversability_topic_local"].GetString(), 10);

    pub_t_score_global_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(p["traversability_topic_global"].GetString(), 10);






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

    // Update global map with new pointcloud data
    for (const auto& p : cloud->points) 
    {
        auto [gx, gy] = grid_manager.pose_to_grid_coordinates(p.x, p.y);
        
        // Bounds check:
        if (gy < 0 || gy >= static_cast<int>(grid_manager.global_grid.size()))
            continue;
        if (gx < 0 || gx >= static_cast<int>(grid_manager.global_grid[0].size()))
            continue;
        
        grid_manager.global_grid[gy][gx].points.push_back(p);
        grid_manager.global_grid[gy][gx].num_points += 1;
    }
};



// =====================================================
// TRANSFORM
// =====================================================


void ROSWrapper::lookupTransform()
{
    try {
        std::cerr << "Looking up transform..." << std::endl;
        rclcpp::Time now = this->get_clock()->now();
        rclcpp::Time safe_time = now - rclcpp::Duration::from_seconds(0.2);  // 50 ms safety margin


        transform_stamped = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      
        robot_coordinates = {transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z};
        robot_coordinates_grid = grid_manager.pose_to_grid_coordinates(robot_coordinates[0], robot_coordinates[1]);
        std::cerr << "Robot coordinates (m): (" << robot_coordinates[0] << ", " << robot_coordinates[1] << ", " << robot_coordinates[2] << ")" << std::endl;

        // Update local map based on new robot position
        this->compute_t_score();

        // std::cerr << "UPDATEE" << std::endl;
        // grid_manager.update_map(robot_coordinates_grid.x, robot_coordinates_grid.y);


        // Publish local and global T-score maps
        std::cerr << "Publishing T-score maps..." << std::endl;
        publish_t_score_map(grid_manager.local_grid, true);
        publish_t_score_map(grid_manager.global_grid, false);

    } 
    catch (const tf2::TransformException & ex) 
    {
        RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
        RCLCPP_WARN(this->get_logger(), "Could not transform 'odom' to 'base_link': %s", ex.what());
        RCLCPP_WARN(this->get_logger(), "Is the TF topic properly published|filled ?");

    }
};

// =====================================================
// DATA PROCESSING
// =====================================================

void ROSWrapper::compute_t_score()
{
    // Re-init local grid (clear roughness / points)
    grid_manager.create_grid(grid_manager.local_grid, local_map_size, local_map_size, resolution);

    const int local_h = static_cast<int>(grid_manager.local_grid.size());
    const int local_w = static_cast<int>(grid_manager.local_grid[0].size());
    const int global_h = static_cast<int>(grid_manager.global_grid.size());
    const int global_w = static_cast<int>(grid_manager.global_grid[0].size());

    const int half_local = local_h / 2;   // since square: local_h == local_w

    const int center_gx = robot_coordinates_grid.x;
    const int center_gy = robot_coordinates_grid.y;

    // For each cell in the LOCAL grid
    for (int ly = 0; ly < local_h; ++ly)
    {
        for (int lx = 0; lx < local_w; ++lx)
        {
            // Map local indices (ly,lx) to global indices (gy,gx)
            int gy = center_gy + (ly);
            int gx = center_gx + (lx);

            // Skip if outside global grid
            if (gy < 0 || gy >= global_h || gx < 0 || gx >= global_w)
                continue;

            TerrainCell& gcell = grid_manager.global_grid[gy][gx];
            TerrainCell& lcell = grid_manager.local_grid[ly][lx];

            if (gcell.num_points >= 3 && gcell.points.size() >= 3)
            {

                // Check if the cell is traversable based on height-span
                double zmin = +1e9;
                double zmax = -1e9;

                for (const auto& p : gcell.points)
                {
                    zmin = std::min(zmin, (double)p.z);
                    zmax = std::max(zmax, (double)p.z);
                }

                double height_span = zmax - zmin;
                gcell.height = height_span;

                // Flag non traversable if height is too large
                gcell.traversable = (height_span < height_thresh);

                if (!gcell.traversable)
                {
                    // mark local copy too
                    lcell = gcell;
                    continue;   // skip plane fitting
                }

                std::vector<double> bestFit;
                t_score.FitPlane(t_score.t, gcell.points, bestFit);
                double mean_z    = t_score.CalculateMeanZ(gcell.points);
                
                if (bestFit.size() < 4 || t_score.distances.empty()) 
                {
                    // couldn't fit a valid plane
                    lcell.roughness  = -1.0;
                    lcell.slope      = -1.0;
                    lcell.mean_z     = mean_z;  // or -1
                    lcell.num_points = gcell.num_points;

                    gcell.roughness  = -1.0;
                    gcell.slope      = -1.0;
                    gcell.mean_z     = mean_z;  // or -1
                    continue;
                }
                

                double roughness = t_score.CalculateRoughness(t_score.distances);
                double slope     = t_score.CalculateSlope(bestFit);

                

                std::cerr << "Cell (" << gx << ", " << gy << ") "
                << "rough=" << roughness
                << " slope=" << slope
                << " mean_z=" << mean_z << std::endl;

                lcell.roughness  = roughness;
                lcell.slope      = slope;
                lcell.mean_z     = mean_z;
                lcell.num_points = gcell.num_points;


                gcell.roughness  = roughness;
                gcell.slope      = slope;
                gcell.mean_z     = mean_z;

            }
            else
            {
                lcell.roughness  = -1.0;
                lcell.slope      = 0.0;
                lcell.mean_z     = 0.0;
                lcell.num_points = 0;

                gcell.roughness  = -1.0;
                gcell.slope      = 0.0;
                gcell.mean_z     = 0.0;
                gcell.num_points = 0;
            }

        }
    }
    // Compute step heights in local grid
    int window_radius_cells = 5;  // 11x11 window
    grid_manager.compute_step_heights_local(window_radius_cells);

}




// =====================================================
// Publisher
// =====================================================

void ROSWrapper::publish_t_score_map(const TerrainGrid &grid, bool is_local)
{
    std::vector vector<double> debug;
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
    occupancy_grid_msg.header.stamp = this->get_clock()->now();
    occupancy_grid_msg.header.frame_id = "map";

    occupancy_grid_msg.info.resolution = resolution;

    int H = grid.size();        // rows
    int W = grid[0].size();     // cols

    occupancy_grid_msg.info.height = H;
    occupancy_grid_msg.info.width  = W;

    double size_x = W * resolution;
    double size_y = H * resolution;

    if (is_local)
    {
        occupancy_grid_msg.info.origin.position.x = robot_coordinates[0] - size_x / 2.0;
        occupancy_grid_msg.info.origin.position.y = robot_coordinates[1] - size_y / 2.0;
    }
    else
    {
        occupancy_grid_msg.info.origin.position.x = -size_x / 2.0;
        occupancy_grid_msg.info.origin.position.y = -size_y / 2.0;
    }

    occupancy_grid_msg.info.origin.position.z = transform_stamped.transform.translation.z;
    occupancy_grid_msg.info.origin.orientation.w = 1.0;

    occupancy_grid_msg.data.assign(W * H, -1);

    if (is_local)
    {   
        std::ofstream outFile("local.csv");
    }else
    {
        std::ofstream outFile("global.csv");
    }


    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int index = y * W + x;

            double result = t_score.calculateTScore(grid[y][x].slope,
                                            grid[y][x].roughness,
                                            grid[y][x].height, grid[y][x].traversable);
            occupancy_grid_msg.data[index] = static_cast<int8_t>(result);
            
            // Open an output file stream (CSV file).
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
            
            // debug.push_back(static_cast<double>({result, x, y, grid[y][x].traversable, grid[y][x].slope, grid[y][x].roughness, grid[y][x].height}));
        }
    }

    std::cout << "Data exported to data.csv" << std::endl;


    std::cerr

    if (is_local)
        pub_t_score_local_->publish(occupancy_grid_msg);
    else
        pub_t_score_global_->publish(occupancy_grid_msg);
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