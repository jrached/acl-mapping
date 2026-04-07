#include <obstacle_tracker/obstacle_tracker_node.h>
#include <limits>

// Adaptive EKF Prediction Step for 3D
void ekf_predict(EKFState &ekf_state, double dt)
{
    int state_size = 9;
    double x = ekf_state.x[0], y = ekf_state.x[1], z = ekf_state.x[2], theta = ekf_state.x[3], phi = ekf_state.x[4], v = ekf_state.x[5], a = ekf_state.x[6], theta_dot = ekf_state.x[7], phi_dot = ekf_state.x[8];
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_size, state_size);
    F(0, 3) = -v * cos(phi) * sin(theta) * dt - 0.5 * a * cos(phi) * sin(theta) * std::pow(dt, 2);
    F(0, 4) = - v * sin(phi) * cos(theta) * dt - 0.5 * a * sin(phi) * cos(theta) * std::pow(dt, 2);
    F(0, 5) = cos(phi) * cos(theta) * dt;
    F(0, 6) = 0.5 * cos(phi) * cos(theta) * std::pow(dt, 2);
    F(1, 3) = v * cos(phi) * cos(theta) * dt + 0.5 * a * cos(phi) * cos(theta) * std::pow(dt, 2);
    F(1, 4) = - v * sin(phi) * sin(theta) * dt - 0.5 * a * sin(phi) * sin(theta) * std::pow(dt, 2);
    F(1, 5) = cos(phi) * sin(theta) * dt;
    F(1, 6) = 0.5 * cos(phi) * sin(theta) * std::pow(dt, 2);
    F(2, 4) = v * cos(phi) * dt + 0.5 * a * cos(phi) * std::pow(dt, 2);
    F(2, 5) = sin(phi) * dt;
    F(2, 6) = 0.5 * sin(phi) * std::pow(dt, 2);
    F(5, 6) = dt;
    F(3, 7) = dt;
    F(4, 8) = dt;

    // Predict state element by element (since can't be framed as a linear operator)
    ekf_state.x[0] = x + v * cos(phi) * cos(theta) * dt + 0.5 * a * cos(phi) * cos(theta) * std::pow(dt, 2); 
    ekf_state.x[1] = y + v * cos(phi) * sin(theta) * dt + 0.5 * a * cos(phi) * sin(theta) * std::pow(dt, 2); 
    ekf_state.x[2] = z + v * sin(phi) * dt + 0.5 * a * sin(phi) * std::pow(dt, 2);
    ekf_state.x[3] = theta + theta_dot * dt; 
    ekf_state.x[4] = phi + phi_dot * dt; 
    ekf_state.x[5] = v + a * dt; 
    ekf_state.x[6] = a; 
    ekf_state.x[7] = theta_dot; 
    ekf_state.x[8] = phi_dot; 

    // Predict covariance
    ekf_state.P = F * ekf_state.P.selfadjointView<Eigen::Lower>() * F.transpose() + ekf_state.Q * dt;
}

// Adaptive EKF Update Step for 3D
void aekf_update(EKFState &ekf_state, const Eigen::VectorXd &z, double alpha, double time_updated, double last_mes_time, const Eigen::Vector3d &bbox, bool use_adaptive_kf)
{
    int state_size = 9;
    Eigen::MatrixXd H; // Measurement matrix (we only measure position [x, y, z])
    H = Eigen::MatrixXd::Zero(3, state_size);
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 2) = 1;

    Eigen::VectorXd d = z - H * ekf_state.x;                           // Measurement residual
    Eigen::MatrixXd S = H * ekf_state.P * H.transpose() + ekf_state.R; // Residual covariance
    Eigen::MatrixXd K = ekf_state.P * H.transpose() * S.inverse();     // Kalman gain

    // Update state
    ekf_state.x = ekf_state.x + K * d;

    // residual
    Eigen::VectorXd epsilon = z - H * ekf_state.x;

    // Adaptive Kalman Filter
    if (use_adaptive_kf)
    {
        ekf_state.R = alpha * ekf_state.R + (1 - alpha) * (epsilon * epsilon.transpose() + H * ekf_state.P * H.transpose());
        ekf_state.Q = alpha * ekf_state.Q + (1 - alpha) * (K * d * d.transpose() * K.transpose());
    }
    else
    {
        ekf_state.R = Eigen::MatrixXd::Identity(3, 3) * 0.01; // For hardware 0.01
        ekf_state.Q = Eigen::MatrixXd::Identity(state_size, state_size) * 0.01; //0.01 For hardware TODO: Make parameter
    }

    // Update covariance
    ekf_state.P = (Eigen::MatrixXd::Identity(state_size, state_size) - K * H) * ekf_state.P; // Update covariance

    // Update time
    ekf_state.time_updated = time_updated;

    // Last time a 3D mesurement was received
    ekf_state.last_mes_time = last_mes_time;

    // Update bounding box
    ekf_state.bbox = 0.5 * ekf_state.bbox + (1 - 0.5) * bbox;

    // Mark as assigned 
    ekf_state.assigned = true; 

}

// Associate cluster with the nearest EKF state using Euclidean distance
int associate_cluster_with_ekf(const Eigen::Vector3d &cluster_centroid, const std::vector<EKFState> &ekf_states, double cluster_tolerance)
{
    double min_distance = cluster_tolerance; // Minimum distance to associate
    int closest_ekf_idx = -1;

    for (int i = 0; i < ekf_states.size(); ++i)
    {
        const Eigen::Vector3d ekf_position(ekf_states[i].x[0], ekf_states[i].x[1], ekf_states[i].x[2]);
        double distance = (cluster_centroid - ekf_position).norm(); // Euclidean distance

        if (distance < min_distance)
        {
            min_distance = distance;
            closest_ekf_idx = i;
        }
    }

    // Return index of the closest EKF state
    return closest_ekf_idx;
}

// Associate gridnet estimate with the nearest 2D EKF state using Euclidean distance
int associate_gridnet_est_with_ekf(const Eigen::Vector3d &gridnet_pos, const std::vector<EKFState> &ekf_states, double gridnet_est_tolerance)
{
    double min_distance = gridnet_est_tolerance; // Minimum distance to associate
    int closest_ekf_idx = -1;

    for (int i = 0; i < ekf_states.size(); ++i)
    {
        const Eigen::Vector3d ekf_2d_position(ekf_states[i].x[0], ekf_states[i].x[1], 0.0);
        double distance = (gridnet_pos - ekf_2d_position).norm(); // Euclidean distance

        if (distance < min_distance)
        {
            min_distance = distance;
            closest_ekf_idx = i;
        }
    }

    // Return index of the closest EKF state
    return closest_ekf_idx;
}

// ObstacleTrackerNode constructor
ObstacleTrackerNode::ObstacleTrackerNode() : Node("obstacle_tracker_node")
{
    // Initialize flags 
    new_gridnet_ = false; 
    gn_counter_ = 0; 

    // Initialize time for kalman filter 
    start_time_ = this->now().seconds();

    RCLCPP_INFO(this->get_logger(), "Obstacle Tracker Node Started");

    // Declare and set parameters
    declareAndsetParameters();

    // Define QoS policy 
    rclcpp::QoS qos_policy(rclcpp::KeepLast(1));
    qos_policy.best_effort();
    qos_policy.durability_volatile();

    rclcpp::QoS cloud_qos(rclcpp::KeepLast(1));
    cloud_qos.reliable();

    // Subscribe to PointCloud2 topic
    sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "point_cloud", cloud_qos, std::bind(&ObstacleTrackerNode::pointcloudCallback, this, std::placeholders::_1));
    
    // Subscribe to GridNet obstacle pose estimates 
    sub_est_obs_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "est_obs_poses", qos_policy, std::bind(&ObstacleTrackerNode::gridnetCallback, this, std::placeholders::_1));

    // Publish clusters and predicted positions
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tracked_obstacles", 10);
    pub_bboxes_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_bounding_boxes", 10);

    // Publish the box's location and associated uncertainty as the sphere size
    pub_unc_sphere_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("uncertainty_spheres", 10);

    // Publish predicted trajectory
    // pub_predicted_traj_ = this->create_publisher<dynus_interfaces::msg::DynTraj>("predicted_trajs", 10);

    // Publish predicted position and velocity 
    pred_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pred_pos", 10);
    pred_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("pred_vel", 10);

    // Initialize the tf2 buffer and listener
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    RCLCPP_INFO(this->get_logger(), "Obstacle Tracker Node Initialized");
}

// Declare and set parameters
void ObstacleTrackerNode::declareAndsetParameters()
{
    // Declare parameters
    this->declare_parameter("visual_level", 1);
    this->declare_parameter("use_adaptive_kf", false);
    this->declare_parameter("adaptive_kf_alpha", 0.98);
    this->declare_parameter("adaptive_kf_dt", 0.1);
    this->declare_parameter("cluster_tolerance", 2.0);
    this->declare_parameter("min_cluster_size", 1);
    this->declare_parameter("max_cluster_size", 2000);
    this->declare_parameter("prediction_horizon", 2.0);
    this->declare_parameter("prediction_dt", 0.1);
    this->declare_parameter("time_to_delete_old_obstacles", 5.0);
    this->declare_parameter("cluster_bbox_cutoff_size", 3.0);
    this->declare_parameter("use_life_time_for_box_visualization", false);
    this->declare_parameter("box_visualization_duration", 3.0);
    this->declare_parameter("dynus_map_res", 0.5);
    this->declare_parameter("velocity_threshold", 0.1);
    this->declare_parameter("acceleration_threshold", 0.1);
    this->declare_parameter("use_hardware", false);
    this->declare_parameter("bbox_density", 1.0);
    this->declare_parameter("use_gridnet", false); 
    this->declare_parameter("alpha", 0.5); 
    this->declare_parameter("gridnet_tolerance", 0.6);
    this->declare_parameter("time_to_hide_obstacle", 0.3);

    // Set parameters
    visual_level_ = this->get_parameter("visual_level").as_int();
    use_adaptive_kf_ = this->get_parameter("use_adaptive_kf").as_bool();
    adaptive_kf_alpha_ = this->get_parameter("adaptive_kf_alpha").as_double();
    adaptive_kf_dt_ = this->get_parameter("adaptive_kf_dt").as_double();
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
    prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
    prediction_dt_ = this->get_parameter("prediction_dt").as_double();
    time_to_delete_old_obstacles_ = this->get_parameter("time_to_delete_old_obstacles").as_double();
    cluster_bbox_cutoff_size_ = this->get_parameter("cluster_bbox_cutoff_size").as_double();
    use_life_time_for_box_visualization_ = this->get_parameter("use_life_time_for_box_visualization").as_bool();
    box_visualization_duration_ = this->get_parameter("box_visualization_duration").as_double();
    dynus_map_res_ = this->get_parameter("dynus_map_res").as_double();
    velocity_threshold_ = this->get_parameter("velocity_threshold").as_double();
    acceleration_threshold_ = this->get_parameter("acceleration_threshold").as_double();
    use_hardware_ = this->get_parameter("use_hardware").as_bool();
    bbox_density_ = this->get_parameter("bbox_density").as_double();
    use_gridnet_ = this->get_parameter("use_gridnet").as_bool(); 
    alpha_ = this->get_parameter("alpha").as_double();
    gridnet_tolerance_ = this->get_parameter("gridnet_tolerance").as_double();
    time_to_hide_obstacle_ = this->get_parameter("time_to_hide_obstacle").as_double(); 

    // Print the parameters
    RCLCPP_INFO(this->get_logger(), "visual_level: %d", visual_level_);
    RCLCPP_INFO(this->get_logger(), "adaptive_kf_alpha: %f", adaptive_kf_alpha_);
    RCLCPP_INFO(this->get_logger(), "adaptive_kf_dt: %f", adaptive_kf_dt_);
    RCLCPP_INFO(this->get_logger(), "cluster_tolerance: %f", cluster_tolerance_);
    RCLCPP_INFO(this->get_logger(), "min_cluster_size: %d", min_cluster_size_);
    RCLCPP_INFO(this->get_logger(), "max_cluster_size: %d", max_cluster_size_);
    RCLCPP_INFO(this->get_logger(), "prediction_horizon: %f", prediction_horizon_);
    RCLCPP_INFO(this->get_logger(), "prediction_dt: %f", prediction_dt_);
    RCLCPP_INFO(this->get_logger(), "time_to_delete_old_obstacles: %f", time_to_delete_old_obstacles_);
    RCLCPP_INFO(this->get_logger(), "cluster_bbox_cutoff_size: %f", cluster_bbox_cutoff_size_);
    RCLCPP_INFO(this->get_logger(), "use_life_time_for_box_visualization: %d", use_life_time_for_box_visualization_);
    RCLCPP_INFO(this->get_logger(), "box_visualization_duration: %f", box_visualization_duration_);
    RCLCPP_INFO(this->get_logger(), "dynus_map_res: %f", dynus_map_res_);
    RCLCPP_INFO(this->get_logger(), "velocity_threshold: %f", velocity_threshold_);
    RCLCPP_INFO(this->get_logger(), "acceleration_threshold: %f", acceleration_threshold_);
    RCLCPP_INFO(this->get_logger(), "use_hardware: %d", use_hardware_);
}

// GridNet callback function 
void ObstacleTrackerNode::gridnetCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    est_obs_poses_ = *msg; 
    new_gridnet_ = true; 
}

// PointCloud2 callback function
void ObstacleTrackerNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pc_timestamp_ = msg->header.stamp;

    start_time_ = this->now().seconds();

    int state_size = 9;

    std::vector<Cluster> clusters;

    // Delete old EKF states that have not been updated for a long time
    deleteOldEKFstates();

    // Mark all EKFs as unnasigned  
    resetEKFassignments(); 

    // Convert PointCloud2 to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Remove NaN values from the cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // Check if the cloud is empty
    if (cloud->empty())
    {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud!");
        return;
    }

    // For hardware, we need to remove slash
    std::string targ_frame_id = msg->header.frame_id; 
    if (use_hardware_ && !targ_frame_id.empty() && targ_frame_id[0] == '/')
        targ_frame_id.erase(0, 1);

    // Transform the cloud to the map frame
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf2_buffer_->lookupTransform(frame_id_,
                                                        targ_frame_id,
                                                        msg->header.stamp,
                                                        rclcpp::Duration::from_seconds(10.0));
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        return;
    }

    // Transform the point cloud to the map frame
    Eigen::Affine3d w_T_b = tf2::transformToEigen(transform_stamped);
    pcl::transformPointCloud(*cloud, *cloud, w_T_b);

    // Set is_dense to true so kdtree search won't fail 
    cloud->is_dense = true; 
    
    // Euclidean Cluster Extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3 * cluster_tolerance_); // Cluster tolerance (distance)
    ec.setMinClusterSize(min_cluster_size_);    // Minimum number of points per cluster
    ec.setMaxClusterSize(max_cluster_size_);    // Maximum number of points per cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Visualize the clustered point cloud
    std::vector<Eigen::Vector3d> cluster_centroids;
    std::vector<Eigen::Vector3d> cluster_bboxes;
    getCentroidsAndSizesOfClusters(cloud, cluster_indices, cluster_centroids, cluster_bboxes);

    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        auto &indices = cluster_indices[i];

        // Get the cluster's centroid as the measurement
        Eigen::Vector3d centroid = cluster_centroids[i];

        // Get the cluster's bounding box size
        Eigen::Vector3d bbox = cluster_bboxes[i];

        // Check bbox size -> if too large, ignore the cluster (probably a static object)        
        if (bbox.norm() > cluster_bbox_cutoff_size_)
        {
            continue;
        }

        // Filter by density
        if (indices.indices.size() / (bbox.norm()) < bbox_density_) {
            continue;
        }

        // Find the closest EKF state (data association)
        int closest_ekf_idx = associate_cluster_with_ekf(centroid, ekf_states_, cluster_tolerance_);

        // Initialize a new cluster
        Cluster cluster;

        if (closest_ekf_idx >= 0)
        {
            // Update the existing EKF state
            ekf_predict(ekf_states_[closest_ekf_idx], this->now().seconds() - ekf_states_[closest_ekf_idx].time_updated);                                                             // EKF Prediction step
            aekf_update(ekf_states_[closest_ekf_idx], centroid, adaptive_kf_alpha_, this->now().seconds(), this->now().seconds(), bbox, use_adaptive_kf_); // EKF Update step
            cluster.setEKFStateAndCentroid(ekf_states_[closest_ekf_idx], centroid);
            ekf_states_[closest_ekf_idx].updateAvgBbox(); //TODO delete
        }
        else
        {
            // No match found, add a new EKF state
            Eigen::MatrixXd Q_avg, R_avg;
            calculateAverageQandR(Q_avg, R_avg);
            EKFState new_state(state_size, Q_avg, R_avg, this->now().seconds(), this->now().seconds(), bbox, ekf_state_id_++, alpha_);
            new_state.x.head(3) = centroid; // Initialize state with the centroid
            ekf_states_.push_back(new_state);
            cluster.setEKFStateAndCentroid(new_state, centroid);
        }

        // Add the cluster to the vector
        clusters.push_back(cluster);
    }
    

    // Check whether EKFs that weren't assigned a MOS measurement can be recoverd through point cloud features
    if (use_gridnet_ && new_gridnet_) 
    {
        for (size_t i = 0; i < est_obs_poses_.poses.size(); ++i) 
        {

            Eigen::Vector3d gridnet_pos(est_obs_poses_.poses[i].position.x, est_obs_poses_.poses[i].position.y, est_obs_poses_.poses[i].position.z);
        
            int closest_ekf_idx = associate_gridnet_est_with_ekf(gridnet_pos, ekf_states_, gridnet_tolerance_);

            if (closest_ekf_idx >= 0 && ekf_states_[closest_ekf_idx].assigned == false && (this->now().seconds() - ekf_states_[closest_ekf_idx].last_mes_time) < 2.0 * time_to_delete_old_obstacles_)
            {
                gn_counter_++; 
                std::cout << "Using Gridnet " << gn_counter_ << std::endl; 

                // Initialize a new cluster
                Cluster cluster;

                // Update EKFs states that didn't find a cluster assignment 
                gridnet_pos[2] = ekf_states_[closest_ekf_idx].x[2];
                ekf_predict(ekf_states_[closest_ekf_idx], this->now().seconds() - ekf_states_[closest_ekf_idx].time_updated); // EKF Prediction step
                aekf_update(ekf_states_[closest_ekf_idx], gridnet_pos, adaptive_kf_alpha_, this->now().seconds(), ekf_states_[closest_ekf_idx].last_mes_time, ekf_states_[closest_ekf_idx].avg_bbox, use_adaptive_kf_); // EKF Update step
                cluster.setEKFStateAndCentroid(ekf_states_[closest_ekf_idx], gridnet_pos);


                // Add the cluster to the vector
                clusters.push_back(cluster);
            }
        }
    }

    // Publish boxes 
    publishBoxes(clusters);

    // Publish predicted positions (using constant acceleration)
    publishPredictions(clusters);

    // Remove static obstacles based on low velocity
    // filterStaticObstacles();

}

void ObstacleTrackerNode::trackObstacles()
{
    
}

// Function to delete old EKF states that have not been updated for a long time
void ObstacleTrackerNode::deleteOldEKFstates()
{
    for (auto it = ekf_states_.begin(); it != ekf_states_.end(); )
    {
        if (start_time_ - it->time_updated > time_to_delete_old_obstacles_)
            it = ekf_states_.erase(it);
        else
            ++it;
    }
}

// Function to calculate the average of Q and R across all EKF states
void ObstacleTrackerNode::calculateAverageQandR(Eigen::MatrixXd &Q_avg, Eigen::MatrixXd &R_avg)
{
    int state_size = 9;
    Q_avg = Eigen::MatrixXd::Zero(state_size, state_size);
    R_avg = Eigen::MatrixXd::Zero(3, 3);

    if (!ekf_states_.empty())
    {
        for (const auto &ekf_state : ekf_states_)
        {
            Q_avg += ekf_state.Q;
            R_avg += ekf_state.R;
        }
        Q_avg /= ekf_states_.size();
        R_avg /= ekf_states_.size();
    }
    else
    {
        // If there are no EKF states, initialize Q and R to default values
        R_avg = Eigen::MatrixXd::Identity(3, 3) * 0.01; // For hardware 0.01
        Q_avg = Eigen::MatrixXd::Identity(state_size, state_size) * 0.01; // For hardware 0.01
    }
}

void ObstacleTrackerNode::getCentroidsAndSizesOfClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<pcl::PointIndices> &cluster_indices, std::vector<Eigen::Vector3d> &cluster_centroids, std::vector<Eigen::Vector3d> &cluster_bboxes)
{

    // Create and add new markers & Get the min/max values for each cluster
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        auto &indices = cluster_indices[i];

        // Use Eigen vectorization
        Eigen::Vector3d min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
        Eigen::Vector3d max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

        for (const auto &idx : indices.indices)
        {
            const auto &point = cloud->points[idx];
            min = min.cwiseMin(Eigen::Vector3d(point.x, point.y, point.z));
            max = max.cwiseMax(Eigen::Vector3d(point.x, point.y, point.z));
        }

        Eigen::Vector3d centroid = (min + max) / 2.0;
        Eigen::Vector3d bbox = max - min;

        // Add centroid and size to the vectors
        cluster_centroids.emplace_back(centroid);
        cluster_bboxes.emplace_back(bbox);
    }
}

void ObstacleTrackerNode::publishBoxes(const std::vector<Cluster> &clusters)
{

    // printf("publishBoxes called with %zu clusters\n", clusters.size());

    visualization_msgs::msg::MarkerArray cluster_markers;
    visualization_msgs::msg::MarkerArray unc_sphere_markers;

    // Set the max scale to avoid very large spheres
    double max_scale = 2.5;

    // Create and add new markers & Get the min/max values for each cluster
    for (auto &cluster : clusters)
    {
        // Initialize position, velocity, and acceleration from EKF state
        Eigen::Vector3d current_position;
        Eigen::Vector3d current_velocity;
        Eigen::Vector3d acceleration;
        double x = cluster.ekf_state.x[0], y = cluster.ekf_state.x[1], z = cluster.ekf_state.x[2], theta = cluster.ekf_state.x[3], phi = cluster.ekf_state.x[4], v = cluster.ekf_state.x[5], a = cluster.ekf_state.x[6];
        current_position = Eigen::Vector3d(x, y, z);
        current_velocity = Eigen::Vector3d(v * cos(phi) * cos(theta), v * cos(phi) * sin(theta), v * sin(phi));
        acceleration = Eigen::Vector3d(a * cos(phi) * cos(theta), a * cos(phi) * sin(theta), a * sin(phi));

        // Create a CUBE marker to visualize the bounding box
        visualization_msgs::msg::Marker marker;
        if (use_life_time_for_box_visualization_)
            marker.lifetime = rclcpp::Duration::from_seconds(box_visualization_duration_);
        marker.header.frame_id = frame_id_;
        // marker.header.stamp = this->now(); // Ensure timestamp consistency
        marker.header.stamp = pc_timestamp_; // Temp. TODO: revert. 
        marker.ns = "cluster_bounding_box";
        marker.id = marker_id_++; // Ensure unique IDs for new markers
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position to the center of the bounding box
        marker.pose.position.x = cluster.ekf_state.x[0];
        marker.pose.position.y = cluster.ekf_state.x[1];
        marker.pose.position.z = cluster.ekf_state.x[2];

        // Set the scale to the size of the bounding box
        marker.scale.x = cluster.ekf_state.bbox[0];
        marker.scale.y = cluster.ekf_state.bbox[1];
        marker.scale.z = cluster.ekf_state.bbox[2];

        // Set the color
        marker.color = cluster.ekf_state.color;

        // Set alpha (transparency)
        marker.color.a = 1.0;

        // Add the bounding box marker to the marker array
        if (current_velocity.norm() > velocity_threshold_ && (start_time_ - cluster.ekf_state.time_updated) < time_to_hide_obstacle_)
        {
            cluster_markers.markers.push_back(marker);
        }

        // Create a SPHERE marker to visualize the uncertainty
        visualization_msgs::msg::Marker unc_sphere_marker;
        unc_sphere_marker.header.frame_id = frame_id_;
        unc_sphere_marker.header.stamp = this->now(); // Ensure timestamp consistency
        unc_sphere_marker.ns = "uncertainty_sphere";
        unc_sphere_marker.id = 0; // Not unique IDs so that only one sphere is visualized
        unc_sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
        unc_sphere_marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position to the center of the bounding box
        unc_sphere_marker.pose.position.x = cluster.centroid[0];
        unc_sphere_marker.pose.position.y = cluster.centroid[1];
        unc_sphere_marker.pose.position.z = cluster.centroid[2];

        // Set the scale to the size of uncertainty
        unc_sphere_marker.scale.x = std::min(cluster.ekf_state.P.diagonal()[0] * 2e3, max_scale);
        unc_sphere_marker.scale.y = std::min(cluster.ekf_state.P.diagonal()[1] * 2e3, max_scale);
        unc_sphere_marker.scale.z = std::min(cluster.ekf_state.P.diagonal()[2] * 2e3, max_scale);

        // Set the color
        unc_sphere_marker.color = cluster.ekf_state.color;

        // Set alpha (transparency)
        unc_sphere_marker.color.a = 0.6;

        // Add the uncertainty sphere marker to the marker array
        if (current_velocity.norm() > velocity_threshold_ && (start_time_ - cluster.ekf_state.time_updated) < time_to_hide_obstacle_)
        {
            unc_sphere_markers.markers.push_back(unc_sphere_marker);
        }
    }

    // Step 3: Publish the marker array to visualize clusters
    pub_bboxes_->publish(cluster_markers);

    // Step 4: Publish the marker array to visualize uncertainty spheres
    pub_unc_sphere_->publish(unc_sphere_markers);
}

// Visualize the predictions over a time horizon, updating both position and velocity
void ObstacleTrackerNode::publishPredictions(const std::vector<Cluster> &clusters)
{

    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    int num_steps = static_cast<int>(prediction_horizon_ / prediction_dt_); // Number of steps

    // Initialize knots and positions
    std::vector<double> t_values;
    std::vector<double> x_values;
    std::vector<double> y_values;
    std::vector<double> z_values;

    for (int i = 0; i < clusters.size(); ++i)
    {

        // Initialize position, velocity, and acceleration from EKF state
        Eigen::Vector3d current_position;
        Eigen::Vector3d initial_position;
        Eigen::Vector3d current_velocity;
        Eigen::Vector3d initial_velocity;
        Eigen::Vector3d acceleration;
        double x = clusters[i].ekf_state.x[0], y = clusters[i].ekf_state.x[1], z = clusters[i].ekf_state.x[2], theta = clusters[i].ekf_state.x[3], phi = clusters[i].ekf_state.x[4], v = clusters[i].ekf_state.x[5], a = clusters[i].ekf_state.x[6];
        current_position = Eigen::Vector3d(x, y, z);
        initial_position = current_position;
        current_velocity = Eigen::Vector3d(v * cos(phi) * cos(theta), v * cos(phi) * sin(theta), v * sin(phi));
        initial_velocity = current_velocity;
        acceleration = Eigen::Vector3d(a * cos(phi) * cos(theta), a * cos(phi) * sin(theta), a * sin(phi));

        // Store the initial position and time
        t_values.push_back(0.0);
        x_values.push_back(current_position[0]);
        y_values.push_back(current_position[1]);
        z_values.push_back(current_position[2]);

        // Ignore low velocities (noise) 
        if (initial_velocity.norm() <= velocity_threshold_)
        {
            continue; 
        }
        
        double t = 0.0;
        for (int step = 0; step < num_steps; ++step)
        {

            for (int j = 0; j < 3; ++j)
            {
                // Avoid high velocity values
                if (current_velocity[j] > 5.0)
                    current_velocity[j] = 5.0;
                if (current_velocity[j] < -5.0)
                    current_velocity[j] = -5.0;
            
                // Avoid high acceleration values
                if (acceleration[j] > 8.0)
                    acceleration[j] = 8.0;
                if (acceleration[j] < -8.0)
                    acceleration[j] = -8.0;
            }

            // Calculate time for this step
            double dt = prediction_dt_;
            t += dt; 

            // Update velocity: v(t) = v_0 + a * t
            Eigen::Vector3d future_velocity = current_velocity + acceleration * dt;

            // Predict future position: p(t) = p_0 + v_0 * t + 0.5 * a * t^2
            Eigen::Vector3d future_position;
            future_position[0] = current_position[0] + current_velocity[0] * dt + 0.5 * acceleration[0] * dt * dt;
            future_position[1] = current_position[1] + current_velocity[1] * dt + 0.5 * acceleration[1] * dt * dt;
            future_position[2] = current_position[2] + current_velocity[2] * dt + 0.5 * acceleration[2] * dt * dt;

            // Store the time and position values
            t_values.push_back(t);
            x_values.push_back(future_position[0]);
            y_values.push_back(future_position[1]);
            z_values.push_back(future_position[2]);

            // Create a marker to visualize the predicted position at this time step
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_; // Ensure it's the correct frame
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set arrow start (current position) and end (future position)
            geometry_msgs::msg::Point start, end;
            start.x = current_position[0];
            start.y = current_position[1];
            start.z = current_position[2];
            end.x = future_position[0];
            end.y = future_position[1];
            end.z = future_position[2];

            // Set arrow start and end points
            marker.points.push_back(start);
            marker.points.push_back(end);

            // Set scale (arrow width and length)
            marker.scale.x = 0.1;
            marker.scale.y = 0.2;

            // Set color (you can gradually fade it based on time step)
            marker.color.r = clusters[i].ekf_state.color.r;
            marker.color.g = clusters[i].ekf_state.color.g;
            marker.color.b = clusters[i].ekf_state.color.b;
            marker.color.a = clusters[i].ekf_state.color.a;

            // Add this marker to the marker array
            markers.markers.push_back(marker);

            // Update the current position and velocity for the next step
            current_position = future_position;
            current_velocity = future_velocity;
        }

        double x_diff = abs(x_values.front() - x_values.back());
        double y_diff = abs(y_values.front() - y_values.back());
        double z_diff = abs(z_values.front() - z_values.back());

        double cutoff_length_threshold = 0.1; // TODO: make this a parameter?

        // If the predicted trajectory is too short, skip the polynomial fitting
        if (x_diff < cutoff_length_threshold && y_diff < cutoff_length_threshold && z_diff < cutoff_length_threshold)
        {
            t_values.clear();
            x_values.clear();
            y_values.clear();
            z_values.clear();
            continue;
        }

        // // Fit a polynomial to the predicted positions
        // Eigen::VectorXd beta_x = polyfit(t_values, x_values, degree_for_pwp_);
        // Eigen::VectorXd beta_y = polyfit(t_values, y_values, degree_for_pwp_);
        // Eigen::VectorXd beta_z = polyfit(t_values, z_values, degree_for_pwp_);

        // // Calculate variance of the residuals
        // double variance_x = calculateVariance(t_values, x_values, beta_x, degree_for_pwp_);
        // double variance_y = calculateVariance(t_values, y_values, beta_y, degree_for_pwp_);
        // double variance_z = calculateVariance(t_values, z_values, beta_z, degree_for_pwp_);

        // // Convert t_values, beta_x, beta_y, beta_z to PieceWisePol
        // PieceWisePol pwp;
        // double current_time = this->now().seconds();
        // pwp.times.push_back(current_time);
        // pwp.times.push_back(current_time + prediction_horizon_);
        // pwp.coeff_x.push_back({beta_x(0), beta_x(1), beta_x(2), beta_x(3)});
        // pwp.coeff_y.push_back({beta_y(0), beta_y(1), beta_y(2), beta_y(3)});
        // pwp.coeff_z.push_back({beta_z(0), beta_z(1), beta_z(2), beta_z(3)});

        // // Fit a quintic polynomial to the predicted positions
        // Eigen::VectorXd beta_x_quintic = polyfit(t_values, x_values, degree_for_poly_);
        // Eigen::VectorXd beta_y_quintic = polyfit(t_values, y_values, degree_for_poly_);
        // Eigen::VectorXd beta_z_quintic = polyfit(t_values, z_values, degree_for_poly_);

        // // Publish DynTraj message with the predicted trajectory
        // dynus_interfaces::msg::DynTraj msg;
        // msg.header.stamp = this->now();
        // msg.header.frame_id = frame_id_;
        // msg.id = clusters[i].ekf_state.id;
        // msg.bbox.push_back(clusters[i].ekf_state.bbox.x());
        // msg.bbox.push_back(clusters[i].ekf_state.bbox.y());
        // msg.bbox.push_back(clusters[i].ekf_state.bbox.z());
        // msg.pwp = dynus_utils::convertPwp2PwpMsg(pwp);
        // msg.ekf_cov_p.push_back(clusters[i].ekf_state.P(0, 0));
        // msg.ekf_cov_p.push_back(clusters[i].ekf_state.P(1, 1));
        // msg.ekf_cov_p.push_back(clusters[i].ekf_state.P(2, 2));
        // msg.ekf_cov_q.push_back(clusters[i].ekf_state.Q(0, 0));
        // msg.ekf_cov_q.push_back(clusters[i].ekf_state.Q(1, 1));
        // msg.ekf_cov_q.push_back(clusters[i].ekf_state.Q(2, 2));
        // msg.ekf_cov_r.push_back(clusters[i].ekf_state.R(0, 0));
        // msg.ekf_cov_r.push_back(clusters[i].ekf_state.R(1, 1));
        // msg.ekf_cov_r.push_back(clusters[i].ekf_state.R(2, 2));
        // msg.poly_cov.push_back(variance_x);
        // msg.poly_cov.push_back(variance_y);
        // msg.poly_cov.push_back(variance_z);

        // // coefficients for quintic polynomial
        // msg.poly_coeffs_x.clear();
        // msg.poly_coeffs_y.clear();
        // msg.poly_coeffs_z.clear();
        // for (int j = 0; j < degree_for_poly_ + 1; ++j)
        // {
        //     msg.poly_coeffs_x.push_back(beta_x_quintic(j));
        //     msg.poly_coeffs_y.push_back(beta_y_quintic(j));
        //     msg.poly_coeffs_z.push_back(beta_z_quintic(j));
        // }

        // // Set the start and end times for the trajectory
        // msg.poly_start_time = current_time;
        // msg.poly_end_time = current_time + prediction_horizon_;

        // msg.is_agent = false;
        // pub_predicted_traj_->publish(msg);

        // Clear the vectors for the next EKF state
        t_values.clear();
        x_values.clear();
        y_values.clear();
        z_values.clear();

        if (initial_velocity.norm() > velocity_threshold_)
        {
            // Publish predicted position 
            geometry_msgs::msg::PoseStamped pred_pos_msg;
            // pred_pos_msg.header.stamp = this->now();
            pred_pos_msg.header.stamp = pc_timestamp_;
            pred_pos_msg.header.frame_id = frame_id_; 
            pred_pos_msg.pose.position.x = initial_position[0];
            pred_pos_msg.pose.position.y = initial_position[1];
            pred_pos_msg.pose.position.z = initial_position[2];
            pred_pos_pub_->publish(pred_pos_msg);

            geometry_msgs::msg::TwistStamped pred_vel_msg;
            // pred_vel_msg.header.stamp = this->now();
            pred_vel_msg.header.stamp = pc_timestamp_;
            pred_vel_msg.header.frame_id = frame_id_; 
            pred_vel_msg.twist.linear.x = initial_velocity[0];
            pred_vel_msg.twist.linear.y = initial_velocity[1];
            pred_vel_msg.twist.linear.z = initial_velocity[2];
            pred_vel_pub_->publish(pred_vel_msg);
        }

    }

    // Publish the marker array with predicted trajectories
    if (visual_level_ >= 0)
        pub_markers_->publish(markers);
}

Eigen::VectorXd ObstacleTrackerNode::polyfit(const std::vector<double> &t, const std::vector<double> &y, int degree)
{

    // Number of data points
    int n = t.size();

    // Construct the Vandermonde matrix for polynomial fitting
    Eigen::MatrixXd X(n, degree + 1);
    Eigen::VectorXd Y(n);

    for (int i = 0; i < n; ++i)
    {
        Y(i) = y[i];
        for (int j = 0; j <= degree; ++j)
        {
            X(i, j) = std::pow(t[i], degree - j); // Note: the coefficients are stored as [a b c d] for a*t^3 + b*t^2 + c*t + d so we need to flip it instead of std::pow(t[i], j)
        }
    }

    // Solve the normal equations: X^T * X * beta = X^T * Y
    Eigen::VectorXd beta = (X.transpose() * X).ldlt().solve(X.transpose() * Y);

    return beta;
}

double ObstacleTrackerNode::calculateVariance(const std::vector<double> &t, const std::vector<double> &y, const Eigen::VectorXd &beta, int degree)
{
    // Calculate residuals and estimate variance
    int n = t.size();
    double residual_sum = 0.0;

    for (int i = 0; i < n; ++i)
    {
        double fitted_value = 0.0;
        for (int j = 0; j <= degree; ++j)
        {
            fitted_value += beta(j) * std::pow(t[i], j);
        }
        double residual = y[i] - fitted_value;
        residual_sum += residual * residual;
    }

    return residual_sum / (n - degree - 1); // variance = sum(residuals^2) / (n - p - 1)
}

// Function to filter out static obstacles based on low velocity
void ObstacleTrackerNode::filterStaticObstacles()
{
    for (auto it = ekf_states_.begin(); it != ekf_states_.end();)
    {
        Eigen::Vector3d velocity = it->x.segment<3>(3); // Extract velocity from the state vector (elements 3,4,5)
        Eigen::Vector3d acceleration = it->x.segment<3>(6); // Extract acceleration from the state vector (elements 6,7,8)
        if (velocity.norm() < velocity_threshold_ && acceleration.norm() < acceleration_threshold_)
        {
            // If the velocity is below the threshold, consider the obstacle static and remove it
            it = ekf_states_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

// GridNet functions 
// Function to reset EKF assignment
void ObstacleTrackerNode::resetEKFassignments()
{
    // Iterate through EKF states setting assigned field to false 
    for (size_t i = 0; i < ekf_states_.size(); ++i)
    {
        ekf_states_[i].assigned = false; 
    }
}


int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    // Initialize multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Create an instance of ObstacleTrackerNode
    auto obstacle_tracker_node = std::make_shared<ObstacleTrackerNode>();
    executor.add_node(obstacle_tracker_node);

    // Spin the executor
    executor.spin();

    rclcpp::shutdown();
    return 0;
}