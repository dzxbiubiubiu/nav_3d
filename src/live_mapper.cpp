
#include "live_mapper.h"

/* ------------------------- Constructor -------------------------
  Attempts to initialize most variables from ros Parameter Server
    Defaults are set for most things, in case yaml isn't loaded\
*/

LiveMapper::LiveMapper() {
	// These variables not used elsewhere in class...
	std::string planar_cloud_topic, poly_topic, height_topic, map_pub_topic, robot_base_frame, viz_topic;

	if (!nh_.param<std::string>("nav_3d/live_mapper/map_pub_topic", map_pub_topic, "nav_3d/live_map"))
		ROS_WARN_STREAM("[Nav_3d] Failed to get map pub topic from parameter server - defaulting to " << map_pub_topic << ".");
	if (!nh_.param<std::string>("nav_3d/live_mapper/planar_cloud_topic", planar_cloud_topic, "laser_stitcher/planar_cloud"))
		ROS_WARN_STREAM("[Nav_3d] Failed to get planar cloud topic from parameter server - defaulting to " << planar_cloud_topic << ".");
	nh_.param<std::string>("nav_3d/live_mapper/poly_topic", poly_topic, "nav_3d/robot_footprint");
	nh_.param<std::string>("nav_3d/live_mapper/height_topic", height_topic, "nav_3d/robot_height");
	nh_.param<std::string>("nav_3d/live_mapper/robot_base_frame", robot_base_frame, "base_footprint");
	nh_.param<int>("nav_3d/live_mapper/lidar_configuration", lidar_config_, 1);
	nh_.param<std::string>("nav_3d/live_mapper/obstacle_algorithm", alg_name_, "slope");
	nh_.param<std::string>("nav_3d/live_mapper/map_registration", map_reg_, "map");
	nh_.param<float>("nav_3d/live_mapper/robot_height_default", robot_height_default_, 1);
	nh_.param<float>("nav_3d/live_mapper/floor_range", floor_range_, 0.1);
	nh_.param<float>("nav_3d/live_mapper/min_obj_dist", min_obj_dist_, 0.0);
    nh_.param<float>("nav_3d/live_mapper/max_check_dist", max_check_dist_, 2.0);
	nh_.param<float>("nav_3d/live_mapper/max_robot_reach", max_robot_reach_, 1.0);
	nh_.param<int>("nav_3d/live_mapper/scan_res", scan_res_, 3);
	nh_.param<float>("nav_3d/live_mapper/map_res", map_res_, 0.05);
	nh_.param<float>("nav_3d/live_mapper/slope_threshold", slope_threshold_, 3);
	nh_.param<float>("nav_3d/live_mapper/drivable_height", drivable_height_, 0.1);
	nh_.param<float>("nav_3d/live_mapper/max_step_height", max_step_height_, 1.0);
	nh_.param<float>("nav_3d/live_mapper/stale_map_time", stale_map_time_, 10);
	nh_.param<float>("nav_3d/live_mapper/obstacle_decay_factor", obs_decay_factor_, 0.9);
	nh_.param<std::string>("nav_3d/live_mapper/obstacle_decay/type", obs_decay_type_, "exponential");
	nh_.param<float>("nav_3d/live_mapper/obstacle_decay/time", obs_decay_time_, 90);
	nh_.param<float>("nav_3d/live_mapper/loop_rate", loop_rate_, 40);
	nh_.param<bool>("nav_3d/live_mapper/visualization_tool/active", viz_tool_, false);
	nh_.param<std::string>("nav_3d/live_mapper/visualization_tool/pub_topic", viz_topic, "nav_3d/obs_cloud");
	nh_.param<float>("nav_3d/live_mapper/visualization_tool/reset_timer", viz_timer_, 10.0);



	if (!(obs_decay_type_ == "exponential" || obs_decay_type_ == "exp" || obs_decay_type_ == "Exponential" || obs_decay_type_ == "EXP" || obs_decay_type_ == "linear" || obs_decay_type_ == "lin" || obs_decay_type_ == "Linear" || obs_decay_type_ == "LIN"))
		ROS_ERROR_STREAM("[Nav_3d] Did not receive a valid obstacle decay type.  All obstacles will accumulate overtime and will never be removed from the map.");

	// Subscribers
	planar_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(planar_cloud_topic, 1, &LiveMapper::mainCallback, this);
	poly_sub_ = nh_.subscribe<geometry_msgs::PolygonStamped>(poly_topic, 1, &LiveMapper::updatePoly, this);
	height_sub_ = nh_.subscribe<geometry_msgs::Point32>(height_topic, 1, &LiveMapper::updateHeight, this);

	// Publishers (publish a occgrid if map output or laserscan if scan output)
	if ((map_reg_ == "map") || (map_reg_ == "Map") || (map_reg_ == "MAP"))
		map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_pub_topic, 1, this);
	else if ((map_reg_ == "scan") || (map_reg_ == "Scan") || (map_reg_ == "SCAN"))
		map_pub_ = nh_.advertise<sensor_msgs::LaserScan>(map_pub_topic, 1, this);
	else
		ROS_ERROR_STREAM("[Nav_3d] Live mapper failed to receive the map registration param.  Aborting live mapper.");

	if (viz_tool_)
		viz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(viz_topic, 1, this);

	// Time keeping
	start_time_ = ros::Time::now();
	prev_map_build_time_ = ros::Time(0);

	// Defaulting the robot height. It will be updated to the true value via updateHeight
	current_robot_height_.z = robot_height_default_;
	robot_base_point_.header.frame_id = robot_base_frame;
	robot_base_point_.header.stamp = ros::Time(0); // Setting to time 0 to allow it to be transformed by tf
	robot_base_point_.point.x = 0;
	robot_base_point_.point.y = 0;
	robot_base_point_.point.z = 0;

	// Initializing
	poly_init_ = false;
	map_init_ = false;
	robot_height_init_ = false;

	ros::Rate loop_rate(loop_rate_);
	ROS_INFO_STREAM("[Nav_3d] LiveMapper initialized successfully.");

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void LiveMapper::mainCallback(const sensor_msgs::PointCloud2::ConstPtr& planar_cloud) {
	if ((map_reg_ == "map") || (map_reg_ == "Map") || (map_reg_ == "MAP")) {
		map_to_publish_.header = planar_cloud->header;

		// Initializing the map if the user wants to register the data in map form
		if (!map_init_) {
			map_to_publish_.info.resolution = map_res_;
			map_to_publish_.info.origin.position.x = 0;
			map_to_publish_.info.origin.position.y = 0;
			map_to_publish_.info.origin.position.z = 0;
			map_to_publish_.info.origin.orientation.x = 0;
			map_to_publish_.info.origin.orientation.y = 0;
			map_to_publish_.info.origin.orientation.z = 0;
			map_to_publish_.info.origin.orientation.w = 1.0;
			map_to_publish_.info.height = 0;
			map_to_publish_.info.width = 0;
			max_x_ = 0;
			max_y_ = 0;
			min_x_ = 0;
			min_y_ = 0;
			map_init_ = true;
		}

		prev_height_ = map_to_publish_.info.height;
		prev_width_ = map_to_publish_.info.width;
		// prev_origin_x_ = map_to_publish_.info.origin.position.x;
		// prev_origin_y_ = map_to_publish_.info.origin.position.y;
		// prev_upper_x_ = map_to_publish_.info.origin.position.x + (map_to_publish_.info.width * map_res_);
		// prev_upper_y_ = map_to_publish_.info.origin.position.y + (map_to_publish_.info.height * map_res_);
		// prev_max_x_ = max_x_;
		// prev_max_y_ = max_y_;
		// prev_min_x_ = min_x_;
		// prev_min_y_ = min_y_;

		prev_map_data_.clear();
		for (int i=0; i<map_to_publish_.data.size(); ++i) {
			prev_map_data_.push_back(map_to_publish_.data[i]);
		}

	} else if ((map_reg_ == "scan") || (map_reg_ == "Scan") || (map_reg_ == "SCAN")) {
		scan_to_publish_.header = planar_cloud->header;

		// Initializing the scan if the user wants to register the data in scan form
		if (!map_init_) {
			scan_to_publish_.angle_min = round(-3.141592653589793 * pow(10, scan_res_))/pow(10, scan_res_); // rounding to the scan_res decimal place;
			scan_to_publish_.angle_max = round(3.141592653589793 * pow(10, scan_res_))/(pow(10, scan_res_)); // rounding to the scan_res decimal place;
			scan_to_publish_.angle_increment = pow(10, -scan_res_);
			num_of_pts_ = (int) ((scan_to_publish_.angle_max - scan_to_publish_.angle_min) / scan_to_publish_.angle_increment); //((round(scan_to_publish_.angle_max * pow(10, scan_res_)) / pow(10, scan_res_) - round(scan_to_publish_.angle_min * pow(10, scan_res_)) / pow(10, scan_res_)) / (round(scan_to_publish_.angle_increment * pow(10, scan_res_)) / pow(10, scan_res_)));
			scan_to_publish_.time_increment = 0;  // Unsure of this but it is likely not needed
			scan_to_publish_.scan_time = 0;  // Unsure of this but it is likely not needed as well
			scan_to_publish_.range_min = 0.01;
			scan_to_publish_.range_max = 30.0;
			map_init_ = true;
			double inf = std::numeric_limits<double>::infinity();

			for (int i=0; i<num_of_pts_; ++i) {
				scan_to_publish_.ranges.push_back(inf);
				scan_to_publish_.intensities.push_back(0);
			}
		}
	}

	if (!robot_height_init_)
		ROS_WARN_THROTTLE(30, "The actual robot height has not been initialized.  Currently working off of the robot height default which is set to %f.", robot_height_default_);
	
	// If the polygon and the points being read are different frames we'll transform the polygon
	if (current_poly_.header.frame_id != planar_cloud->header.frame_id)
		this->convertPoly(planar_cloud->header);
	
	// Converting the planar cloud into a XYZ cloud before running any algorithms
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*planar_cloud, *pcl_cloud);


	// Calling the right algorithm to run on the planar_cloud to find the obstacles
	if (alg_name_ == "height" || alg_name_ == "Height" || alg_name_ == "HEIGHT" || alg_name_ == "height_method" || alg_name_ == "height method" || alg_name_ == "HEIGHT METHOD" || alg_name_ == "Height Method") {
		points_checked_.clear();
		this->heightMethod(pcl_cloud);

		// Since we have the new size of the map we can assign proper cell values to each point in cloud
		this->mapDiscretizer();

	} else if (alg_name_ == "slope" || alg_name_ == "Slope" || alg_name_ == "SLOPE" || alg_name_ == "slope_method" || alg_name_ == "slope method" || alg_name_ == "SLOPE METHOD" || alg_name_ == "Slope Method") {
		// Transforming the robot_base_point from the robot_base_frame to the same frame as the planar_cloud (this only needs to be done for the slope method)
		try {
			listener_.transformPoint(planar_cloud->header.frame_id, robot_base_point_, robot_base_point_);
		} catch (const tf::TransformException& e) {
			robot_base_point_.header.stamp = ros::Time::now(); // Base point could not be transformed so let's ensure it has the correct time stamp
			ROS_ERROR_THROTTLE(30, "[Nav_3d] Live Mapper could not get the transform from the map frame to the base of the robot.  Slope method will not always output accurate obstacle data is this transform isn't known. If this error persists you might want to use height method for obstacle detection.");
			// ROS_ERROR_STREAM("[Nav_3d] Live Mapper could not get the transform from the map frame to the base of the robot.  Slope method will not always output accurate obstacle data is this transform isn't known. If this error persists you might want to use height method for obstacle detection.");
		}

		// Before slope method we have to parse the raw cloud understanding the lidar config
		this->cloudParser(pcl_cloud);

		// Since we have the new size of the map we can assign proper cell values to each point in cloud
		this->mapDiscretizer();

		points_checked_.clear();
		this->slopeMethod(pcl_cloud);
	} else {
		ROS_ERROR_STREAM("[Nav_3d] Failed to receive an algorithm to run.  Cannot perform obstacle detection. Check the yaml algorithm entry to ensure it is typed correctly.");
	}

	// Build the map now
	if ((map_reg_ == "map") || (map_reg_ == "Map") || (map_reg_ == "MAP"))
		this->mapBuilder();
	else if ((map_reg_ == "scan") || (map_reg_ == "Scan") || (map_reg_ == "SCAN"))
		this->scanBuilder();

	// Publish
	if ((map_reg_ == "map") || (map_reg_ == "Map") || (map_reg_ == "MAP"))
		map_pub_.publish(map_to_publish_);
	else if ((map_reg_ == "scan") || (map_reg_ == "Scan") || (map_reg_ == "SCAN"))
		map_pub_.publish(scan_to_publish_);

	// Visualize data in point cloud form if viz tool is set to active
	if (viz_tool_)
		this->visualizationTool();
}

void LiveMapper::heightMethod(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& planar_cloud) {
	geometry_msgs::PointStamped cloud_point;
	// cloud_point.header.stamp = planar_cloud->header.stamp;
	cloud_point.header.frame_id = planar_cloud->header.frame_id;
	
	point_XYZDTO checked_point;
	for (int i=0; i<planar_cloud->points.size(); ++i) {
		if (planar_cloud->points[i].z < current_robot_height_.z && (sqrt(planar_cloud->points[i].x * planar_cloud->points[i].x + planar_cloud->points[i].y * planar_cloud->points[i].y) > min_obj_dist_)) {
			this->mapSizeMaintainer(planar_cloud->points[i]);
			
			checked_point.x = planar_cloud->points[i].x;
			checked_point.y = planar_cloud->points[i].y;
			checked_point.z = planar_cloud->points[i].z;
			checked_point.distance = sqrt(checked_point.x * checked_point.x + checked_point.y * checked_point.y);
			checked_point.time_stamp = ros::Time::now();
			checked_point.obstacle = 0;
			
			// This is how the height method determines what the ground is
			if (!(planar_cloud->points[i].z > - floor_range_ && planar_cloud->points[i].z <  floor_range_)) {
		
				if (poly_init_) {
					cloud_point.point.x = planar_cloud->points[i].x;
					cloud_point.point.y = planar_cloud->points[i].y;

					// Check to see if the points is within the robot
					// This is made efficient by first checking to make sure the point is within the robot possible max reach
					if (checked_point.distance < max_robot_reach_) {
						in_poly_ = point_in_poly(current_poly_, cloud_point);

						// If the point is not in the polygon then we build it in as an obstacle					
						if (!in_poly_) {
							checked_point.obstacle = 1;
							occupied_list_.push_back(checked_point);
						}
					}
				} else {
					checked_point.obstacle = 1;
					occupied_list_.push_back(checked_point);
				}
			}
		
            points_checked_.push_back(checked_point);
		}
	}
}

void LiveMapper::slopeMethod(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& planar_cloud) {
	geometry_msgs::PointStamped cloud_point;
	// cloud_point.header.stamp = planar_cloud->header.stamp;
	cloud_point.header.frame_id = planar_cloud->header.frame_id;

	// Perform the slope method calculation on the front cloud
	// NOTE it is important that the first assumed ground point is at 0
	// and we begin the first point to analyze at 1
	float slope;
	int g = 0;
	for (int i=1; i<front_cloud_.size(); ++i) {
		// /Ignore all points that are higher than the highest point in the robot
		if (front_cloud_[i].z < current_robot_height_.z && front_cloud_[i].distance > min_obj_dist_) {

            // Check to see if the point is within the robot footprint
            // This is made efficient by first checking to make sure the point is within the robot possible max reach
            if (poly_init_ && front_cloud_[i].distance < max_robot_reach_) {
                cloud_point.point.x = front_cloud_[i].x;
                cloud_point.point.y = front_cloud_[i].y;
                in_poly_ = point_in_poly(current_poly_, cloud_point);
            } else {
                // Even if the polygon is not initialized 
                in_poly_ = false;
            }

            // If the point is not inside the polygon then we are going to check to see if it is an obstacle point
            if ((!in_poly_) && ((front_cloud_[i].distance - front_cloud_[g].distance) < max_check_dist_)) {

                // Slope calculation and check
                slope = (front_cloud_[i].z - front_cloud_[g].z) / (front_cloud_[i].distance - front_cloud_[g].distance);
                if (fabs(slope) > slope_threshold_) {

                    // If the point is flagged by having a sharp slope it can still be in the likely drivable region if it is within the drivable height
                    if (fabs(front_cloud_[i].z - front_cloud_[g].z) > drivable_height_) {
                        front_cloud_[i].obstacle = 1;
                        occupied_list_.push_back(front_cloud_[i]);
                    } else {
                    	// Likely drivable point
                    	front_cloud_[i].obstacle = 2;
                    }

                // If the point has passed the slope test but is at a great height relative to the last ground point then we aren't sure it is the ground
                } else if (fabs(front_cloud_[i].z - front_cloud_[g].z) > max_step_height_){
                    front_cloud_[i].obstacle = 3;

                // If the point passes the slope and the height tests then the new furthest ground point is at the current index
                } else {
                    g = i;
                }
                // Accumulating a vector of all the points that have been checked in this callback
                points_checked_.push_back(front_cloud_[i]);
            }
		}
	}

	// Perform the slope method calculation on the front cloud
	// NOTE it is important that the first assumed ground point is at 0
	// and we begin the first point to analyze at 1
	g = 0;
	for (int i=1; i<back_cloud_.size(); ++i) {
		// /Ignore all points that are higher than the highest point in the robot
		if (back_cloud_[i].z < current_robot_height_.z && back_cloud_[i].distance > min_obj_dist_) {

            // Check to see if the point is within the robot footprint
            // This is made efficient by first checking to make sure the point is within the robot possible max reach
            if (poly_init_ && back_cloud_[i].distance < max_robot_reach_) {
                cloud_point.point.x = back_cloud_[i].x;
                cloud_point.point.y = back_cloud_[i].y;
                in_poly_ = point_in_poly(current_poly_, cloud_point);
            } else {
                // Even if the polygon is not initialized 
                in_poly_ = false;
            }

            // If the point is not inside the polygon then we are going to check to see if it is an obstacle point
            if ((!in_poly_) && ((back_cloud_[i].distance - back_cloud_[g].distance) < max_check_dist_)) {

                // Slope calculation and check
                slope = (back_cloud_[i].z - back_cloud_[g].z) / (back_cloud_[i].distance - back_cloud_[g].distance);
                if (fabs(slope) > slope_threshold_) {

                    // If the point is flagged by having a sharp slope it can still be in the likely drivable region if it is within the drivable height
                    if (fabs(back_cloud_[i].z - back_cloud_[g].z) > drivable_height_) {
                        back_cloud_[i].obstacle = 1;
                        occupied_list_.push_back(back_cloud_[i]);
                    } else {
                    	// Likely drivable point
                    	back_cloud_[i].obstacle = 2;
                    }

                // If the point has passed the slope test but is at a great height relative to the last ground point then we aren't sure it is the ground
                } else if (fabs(back_cloud_[i].z - back_cloud_[g].z) > max_step_height_){
                    back_cloud_[i].obstacle = 3;

                // If the point passes the slope and the height tests then the new furthest ground point is at the current index
                } else {
                    g = i;
                }
                // Accumulating a vector of all the points that have been checked in this callback
                points_checked_.push_back(back_cloud_[i]);
            }
		}
	}
}

void LiveMapper::cloudParser(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& planar_cloud) {
	front_cloud_.clear();
	back_cloud_.clear();

	// Lidar pointed upward in which you get front and back clouds
	if (lidar_config_ == 1) {
		int mid_index, mid_index_2;
		// If the size of the point vector is odd we will capture the two points that strattle the middle
		if (planar_cloud->width % 2 != 0) {
			mid_index = (planar_cloud->width - 1) / 2;
			mid_index_2 = (planar_cloud->width + 1) / 2;
		} else {
			// If it is even then we will just capture the mid_index for later use
			mid_index = mid_index_2 = planar_cloud->width / 2;
		}

		// Building the front half of the cloud for analysis
		front_cloud_.resize(mid_index);
		for (int i=0; i<mid_index; ++i) {
			this->mapSizeMaintainer(planar_cloud->points[i]);

			// Need to subtract the xyz coordinates of the robot_base_frame to tranlate it into the robot_base_frame
			front_cloud_[i].x = planar_cloud->points[i].x - robot_base_point_.point.x;
			front_cloud_[i].y = planar_cloud->points[i].y - robot_base_point_.point.y;
			front_cloud_[i].z = planar_cloud->points[i].z - robot_base_point_.point.z;
			front_cloud_[i].distance = sqrt(front_cloud_[i].x * front_cloud_[i].x + front_cloud_[i].y * front_cloud_[i].y);		
			front_cloud_[i].time_stamp = ros::Time::now();
			front_cloud_[i].obstacle = 0;
		}

		// Adding an all zeros robot base point and then sorting the front cloud by distance
		point_XYZDTO zero_base_point;
		zero_base_point.x = 0;
		zero_base_point.y = 0;
		zero_base_point.z = 0;
		zero_base_point.distance = 0;
		zero_base_point.time_stamp = ros::Time::now();
		front_cloud_.push_back(zero_base_point);
		std::sort(front_cloud_.begin(), front_cloud_.end(), compareDistance);

		// Building the back half of the cloud for analysis
		back_cloud_.resize(mid_index);
		for (int i=0; i<mid_index; ++i) {
			this->mapSizeMaintainer(planar_cloud->points[i + mid_index_2]);

			// Need to subtract the xyz coordinates of the robot_base_frame to tranlate it into the robot_base_frame
			back_cloud_[i].x = planar_cloud->points[i + mid_index_2].x - robot_base_point_.point.x;
			back_cloud_[i].y = planar_cloud->points[i + mid_index_2].y - robot_base_point_.point.y;
			back_cloud_[i].z = planar_cloud->points[i + mid_index_2].z - robot_base_point_.point.z;
			back_cloud_[i].distance = sqrt(back_cloud_[i].x * back_cloud_[i].x + back_cloud_[i].y * back_cloud_[i].y);
			back_cloud_[i].time_stamp = ros::Time::now();
			back_cloud_[i].obstacle = 0;
		}

		// Adding an all zeros robot base point and then sorting the back cloud by distance
		back_cloud_.push_back(zero_base_point);
		std::sort(back_cloud_.begin(), back_cloud_.end(), compareDistance);

	// Lidar pointed forward only getting a front cloud
	} else if (lidar_config_ == 2) {
		int cloud_size = planar_cloud->points.size();
		// Building the front half of the cloud for analysis
		std::vector<point_XYZDTO> front_cloud_(cloud_size);
		for (int i=0; i<cloud_size; ++i) {
			this->mapSizeMaintainer(planar_cloud->points[i]);

			// Need to subtract the xyz coordinates of the robot_base_frame to tranlate it into the robot_base_frame
			front_cloud_[i].x = planar_cloud->points[i].x - robot_base_point_.point.x;
			front_cloud_[i].y = planar_cloud->points[i].y - robot_base_point_.point.y;
			front_cloud_[i].z = planar_cloud->points[i].z - robot_base_point_.point.z;
			front_cloud_[i].distance = sqrt(front_cloud_[i].x * front_cloud_[i].x + front_cloud_[i].y * front_cloud_[i].y);		
			front_cloud_[i].time_stamp = ros::Time::now();
			front_cloud_[i].obstacle = 0;
		}

		// Adding an all zeros robot base point and then sorting the front cloud by distance
		point_XYZDTO zero_base_point;
		zero_base_point.x = 0;
		zero_base_point.y = 0;
		zero_base_point.z = 0;
		zero_base_point.distance = 0;
		zero_base_point.time_stamp = ros::Time::now();
		front_cloud_.push_back(zero_base_point);
		std::sort(front_cloud_.begin(), front_cloud_.end(), compareDistance);
	
	} else if (lidar_config_ == 3) {
		// Reserved for 3D lidars
	}
}

// The Map Builder function contructs an OccupancyGrid map based on the class varibale occupied_list and new_obs.
// New_obs is strictly the new vector of obstacle points that have been added in this most recent spin
void LiveMapper::mapBuilder() {
	bool stale_map;
	if(prev_map_build_time_ == ros::Time(0))
		stale_map = true;
	else if(ros::Duration(ros::Time::now() - prev_map_build_time_).toSec() > stale_map_time_)
		stale_map = true;
	else
		stale_map = false;

	// If the map dimensions have changed at all we need to transfer the data over from the old map into the new map
	if (map_dim_change_) {
		int new_map_size = map_to_publish_.info.width * map_to_publish_.info.height;

		// If the map incresed in size in any of these three directions then we need to create a new fresh map and remap the previous data over
		if (!(dx_ == 0 && d_x_ == 0 && d_y_ == 0)) {
			// Creating a new fresh map of all unknowns at the new bigger size.
			map_to_publish_.data.clear();
			for (int i=0; i<new_map_size; ++i) {
				map_to_publish_.data.push_back(-1);
			}

			// Transfering the old data into the new map
			int new_index;
			for (int i=0; i<prev_map_data_.size(); ++i) {
				// Super smart indexing formula to shift the old data to the new fresh map
				new_index = i + d_y_ * (map_to_publish_.info.width) + ((int)(i / prev_width_) + 1) * d_x_ + ((int)(i / prev_width_)) * dx_;

				if(prev_map_data_[i] <= 0)
					map_to_publish_.data[new_index] = prev_map_data_[i];
				else
					map_to_publish_.data[new_index] = prev_map_data_[i];
			}
        // If the map only changed size in the positive y direction then we can just add on unkown points to the end of the data
		} else {
			while (map_to_publish_.data.size() < new_map_size) {
				map_to_publish_.data.push_back(-1);
			}
		}
	}

	int y_placement, x_placement, placement_spot;
	std::vector<int> cells_checked;
	cells_checked.clear();

	// Running throught the new points that have been checked
	for (int i=0; i<points_checked_.size(); ++i) {

		// Only going to build in points that are definitely not an obstacle (0) or definitely an obstacle (1) into the map
		if (points_checked_[i].obstacle < 2) {

			// Placement spots to determine where to put the data in the map
			x_placement = (int) round((points_checked_[i].x - map_to_publish_.info.origin.position.x) / map_res_);
			y_placement = (int) round((points_checked_[i].y - map_to_publish_.info.origin.position.y) / map_res_);
			placement_spot = (y_placement) * map_to_publish_.info.width + x_placement;

			// If the point checked is a new obstacle then set it to 99 on the map
			if (points_checked_[i].obstacle == 1) {
				map_to_publish_.data[placement_spot] = 99;
				cells_checked.push_back(placement_spot);
	        // If the point checked is previously unknown and the point is not flagged as an obstacle then it will be assigned as a ground point
			} else if (map_to_publish_.data[placement_spot] <= 0 && (! (points_checked_[i].obstacle == 1) )) {
				map_to_publish_.data[placement_spot] = 0;
	        // do the obstacle decay 
			} else {
				// If the cell previously was occupied with an obstacle and has been checked and there has not been a new obstacle seen
				// we will reduce the value of the probability that an obstacle is in that cell by multiplying the obstacle decay factor (< 1).
				if (std::find(cells_checked.begin(), cells_checked.end(), placement_spot) == cells_checked.end()) {
					map_to_publish_.data[placement_spot] = map_to_publish_.data[placement_spot] * obs_decay_factor_;
					cells_checked.push_back(placement_spot);
				}
			}			
		}
	}

	prev_map_build_time_ = ros::Time::now();
}

// The Scan Builder function contructs a feaux 2D laserscan based on the class varibale occupied_list.
void LiveMapper::scanBuilder() {
	int obs_prob, placement_spot;

	for (int i=0; i<occupied_list_.size(); ++i) {
		if (obs_decay_type_ == "exponential" || obs_decay_type_ == "exp" || obs_decay_type_ == "Exponential" || obs_decay_type_ == "EXP")
			obs_prob = std::min((int)(99 * exp(-(ros::Duration(ros::Time::now() - occupied_list_[i].time_stamp).toSec()) / obs_decay_time_ )) , 99);
			// Modeled like a decay function N0 * e ^ -t / Tau where Tau (obs_decay_time) is half life
		else if (obs_decay_type_ == "linear" || obs_decay_type_ == "lin" || obs_decay_type_ == "Linear" || obs_decay_type_ == "LIN")
			obs_prob = std::min((int)(99 - 99 * (ros::Duration(ros::Time::now() - occupied_list_[i].time_stamp).toSec()) / obs_decay_time_ ) , 99);
			// Modeled linearly where obs_decay_time is the time for the obstacle to reach 0 probability

		// If the probability of an obstacle is greater than 5% we will build it into the map
		if (obs_prob > 5) {
			if (!(occupied_list_[i].x < min_x_ || occupied_list_[i].x > max_x_ || occupied_list_[i].y < min_y_ || occupied_list_[i].y > max_y_)) {
				placement_spot = (int) ((scan_to_publish_.angle_max + (round((std::atan2(occupied_list_[i].y, occupied_list_[i].x)) * pow(10, scan_res_))) / pow(10, scan_res_)) / scan_to_publish_.angle_increment) - 1;
				if (placement_spot < scan_to_publish_.ranges.size()) {
					scan_to_publish_.ranges[placement_spot] = occupied_list_[i].distance;
					scan_to_publish_.intensities[placement_spot] = obs_prob * 40; // Intensity scales up to 4000 approximately
				} else {
					occupied_list_.erase(occupied_list_.begin() + i);
				}
			}
		}
	}
}

void LiveMapper::mapSizeMaintainer(const pcl::PointXYZI point) {
	if(point.x > max_x_)
		max_x_ = point.x + 5 * map_res_;
	if(point.y > max_y_)
		max_y_ = point.y + 5 *map_res_;
	if(point.x < min_x_)
		min_x_ = point.x - 5 *map_res_;
	if(point.y < min_y_)
		min_y_ = point.y - 5 * map_res_;
}

void LiveMapper::mapDiscretizer() {
    dx_ = 0; 
    dy_ = 0;
    d_x_ = 0; 
    d_y_ = 0;

	// Increasing any necessary dimensions of the map so that it contains every point that has been measured
	while (map_to_publish_.info.origin.position.x > min_x_) {
		map_to_publish_.info.origin.position.x -= map_res_;
        ++map_to_publish_.info.width;
        ++d_x_;
	}
	while (map_to_publish_.info.origin.position.y > min_y_) {
		map_to_publish_.info.origin.position.y -= map_res_;
        ++map_to_publish_.info.height;
        ++d_y_;
	}

	while ((map_to_publish_.info.origin.position.x + (map_to_publish_.info.width * map_res_)) < max_x_) {
		++map_to_publish_.info.width;
        ++dx_;
	}
	while ((map_to_publish_.info.origin.position.y + (map_to_publish_.info.height * map_res_)) < max_y_) {
		++map_to_publish_.info.height;
        ++dy_;
	}

	// upper_x_ = map_to_publish_.info.origin.position.x + (map_to_publish_.info.width * map_res_);
	// upper_y_ = map_to_publish_.info.origin.position.y + (map_to_publish_.info.height * map_res_);

	// ROS_INFO_STREAM("Width " << map_to_publish_.info.width << " Height " << map_to_publish_.info.height);

	// Checking to see if the size of the map has changed.  No need to refill an entire new map if it is the same size
	map_dim_change_ = (prev_width_ != map_to_publish_.info.width) || (prev_height_ != map_to_publish_.info.height);
}

void LiveMapper::convertPoly(const std_msgs::Header new_header) {
	geometry_msgs::PointStamped temp_poly_point;

	for (int i=0; i<current_poly_.polygon.points.size(); ++i) {
		temp_poly_point.header = current_poly_.header;
		temp_poly_point.point.x = current_poly_.polygon.points[i].x;
		temp_poly_point.point.y = current_poly_.polygon.points[i].y;
		temp_poly_point.point.z = current_poly_.polygon.points[i].z;
		try {
			listener_.transformPoint(new_header.frame_id, temp_poly_point, temp_poly_point);
			
			current_poly_.polygon.points[i].x = temp_poly_point.point.x;
			current_poly_.polygon.points[i].y = temp_poly_point.point.y;
			current_poly_.polygon.points[i].z = temp_poly_point.point.z;
		} catch (const tf::TransformException& e) {
			ROS_ERROR_THROTTLE(30, "[Nav_3d] Live Mapper could not convert the robot polygon into the same frame as the incoming cloud scan.  It is likely that obstacle data around the robot will be inaccurate.");
		}

	}

	current_poly_.header.frame_id = new_header.frame_id;
}

void LiveMapper::updatePoly(const geometry_msgs::PolygonStamped::ConstPtr& new_poly_stamped) {
	current_poly_ = *new_poly_stamped;
	poly_init_ = true;
}

void LiveMapper::updateHeight(const geometry_msgs::Point32::ConstPtr& new_height) {
	current_robot_height_ = *new_height;
	robot_height_init_ = true;
}

void LiveMapper::visualizationTool() {
	pcl::PointXYZI point;

	// Timer or rotation check
	if (prev_viz_pub_time_ == ros::Time(0)) {
		
		viz_cloud_.height = 1;
		viz_cloud_.header.frame_id = "map";

		// Initiating a point with the highest intesity value to maintain the color scale
		point.x = 0;
		point.y = 0;
		point.z = 0;
		point.intensity = 3;
		++viz_cloud_.width;
		viz_cloud_.points.push_back(point);

	} else if (ros::Duration(ros::Time::now() - prev_viz_pub_time_).toSec() > viz_timer_) {

		viz_cloud_.width = 0;
		viz_cloud_.points.clear();

		// Initiating a point with the highest intesity value to maintain the color scale
		point.x = 0;
		point.y = 0;
		point.z = 0;
		point.intensity = 3;
		++viz_cloud_.width;
		viz_cloud_.points.push_back(point);

	}

	// Adding points to the viz cloud
	for (int i=0; i<points_checked_.size(); ++i) {
		++viz_cloud_.width;
		
		// intensity_val = points_checked_[i].obstacle * 100.0 / 1.1;
		point.x = points_checked_[i].x;
		point.y = points_checked_[i].y;
		point.z = points_checked_[i].z;
		point.intensity = points_checked_[i].obstacle;

		
		viz_cloud_.points.push_back(point);
	}

	pcl::toROSMsg(viz_cloud_, viz_cloud_pub_);
	viz_pub_.publish(viz_cloud_pub_);

	prev_viz_pub_time_ = ros::Time::now();
}


bool LiveMapper::compareDistance(point_XYZDTO p1, point_XYZDTO p2) {
	return (p1.distance < p2.distance);
}

int main(int argc, char** argv) {
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	ros::init(argc, argv, "nav_3d/live_mapper");

	LiveMapper live_mapper;
}