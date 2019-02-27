
///////////////////////////////////////////////////////////////////////////////
//      Title     : post_mapper.cpp
//      Project   : nav_3d
//      Created   : 1/1/2019
//      Author    : Chris Suarez
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2018. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////


#include "post_mapper.h"

/* ------------------------- Constructor -------------------------
  Attempts to initialize most variables from ros Parameter Server
    Defaults are set for most things, in case yaml isn't loaded\
*/

PostMapper::PostMapper() {
	// These variables not used elsewhere in class...
	std::string cloud_topic, poly_topic, height_topic, map_topic, map_pub_topic, scan_pub_topic, viz_topic;

	if (!nh_.param<std::string>("nav_3d/post_mapper/map_pub_topic", map_pub_topic, "nav_3d/post_map")) {
		ROS_WARN_STREAM("[Nav_3d] Failed to get map pub topic from parameter server - defaulting to " << map_pub_topic << ".");
	}
	if (!nh_.param<std::string>("nav_3d/post_mapper/scan_pub_topic", scan_pub_topic, "nav_3d/post_scan")) {
		ROS_WARN_STREAM("[Nav_3d] Failed to get map pub topic from parameter server - defaulting to " << scan_pub_topic << ".");
	}
	if (!nh_.param<std::string>("nav_3d/post_mapper/cloud_topic", cloud_topic, "nav_3d/viz_cloud")) {
		ROS_WARN_STREAM("[Nav_3d] Failed to get cloud topic from parameter server - defaulting to " << cloud_topic << ".");
	}

	nh_.param<std::string>("nav_3d/post_mapper/poly_topic", poly_topic, "nav_3d/robot_footprint_stamped");
	nh_.param<std::string>("nav_3d/post_mapper/height_topic", height_topic, "nav_3d/robot_height");
	nh_.param<std::string>("nav_3d/post_mapper/map_topic", map_topic, "nav_3d/live_map");
	nh_.param<std::string>("nav_3d/post_mapper/robot_base_frame", robot_base_frame_, "/base_footprint");
	nh_.param<std::string>("nav_3d/post_mapper/obstacle_algorithm", alg_name_, "slope");
	nh_.param<std::string>("nav_3d/post_mapper/map_registration", map_reg_, "map");
	nh_.param<float>("nav_3d/post_mapper/robot_height_default", robot_height_default_, 1);
	nh_.param<float>("nav_3d/post_mapper/robot_height_buffer", robot_height_buffer_, 0.1);
	nh_.param<float>("nav_3d/post_mapper/robot_height_warning", robot_height_warning_, 0.25);
	nh_.param<float>("nav_3d/post_mapper/floor_range", floor_range_, 0.1);
	nh_.param<float>("nav_3d/post_mapper/min_obj_dist", min_obj_dist_, 0.0);
    nh_.param<float>("nav_3d/post_mapper/max_check_dist", max_check_dist_, 2.0);
	nh_.param<float>("nav_3d/post_mapper/max_robot_reach", max_robot_reach_, 1.0);
	nh_.param<int>("nav_3d/post_mapper/scan_res", scan_res_, 3);
	nh_.param<float>("nav_3d/post_mapper/map_res", map_res_, 0.05);
	nh_.param<float>("nav_3d/post_mapper/slope_threshold", slope_threshold_, 3);
	nh_.param<float>("nav_3d/post_mapper/drivable_height", drivable_height_, 0.1);
	nh_.param<float>("nav_3d/post_mapper/max_step_height", max_step_height_, 1.0);
	nh_.param<float>("nav_3d/post_mapper/max_ditch_depth", max_ditch_depth_, 1.0);
	nh_.param<float>("nav_3d/post_mapper/stale_map_time", stale_map_time_, 10);
	nh_.param<float>("nav_3d/post_mapper/obstacle_decay_factor", obs_decay_factor_, 0.8);
	nh_.param<std::string>("nav_3d/post_mapper/obstacle_decay/type", obs_decay_type_, "exponential");
	nh_.param<float>("nav_3d/post_mapper/obstacle_decay/time", obs_decay_time_, 90);
	nh_.param<float>("nav_3d/post_mapper/recent_obstacle_purge_time", recent_obs_purge_time_, 5);
	nh_.param<float>("nav_3d/post_mapper/loop_rate", loop_rate_, 40);
	nh_.param<bool>("nav_3d/post_mapper/visualization_tool/active", viz_tool_, false);
	nh_.param<std::string>("nav_3d/post_mapper/visualization_tool/pub_topic", viz_topic, "nav_3d/obs_cloud");
	nh_.param<float>("nav_3d/post_mapper/visualization_tool/reset_timer", viz_timer_, 10.0);

	nh_.param<float>("nav_3d/post_mapper/parsing_segments", parsing_segments_, 360);



	if (!(obs_decay_type_ == "exponential" || obs_decay_type_ == "exp" || obs_decay_type_ == "Exponential" || obs_decay_type_ == "EXP" || obs_decay_type_ == "linear" || obs_decay_type_ == "lin" || obs_decay_type_ == "Linear" || obs_decay_type_ == "LIN")) {
		ROS_ERROR_STREAM("[Nav_3d] Did not receive a valid obstacle decay type.  All obstacles will accumulate overtime and will never be removed from the map.");
	}

	// Subscribers
	cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 1, &PostMapper::mainCallback, this);
	map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, &PostMapper::updateMap, this);
	poly_sub_ = nh_.subscribe<geometry_msgs::PolygonStamped>(poly_topic, 1, &PostMapper::updatePoly, this);
	height_sub_ = nh_.subscribe<geometry_msgs::Point32>(height_topic, 1, &PostMapper::updateHeight, this);

	// Publishers (publish a occgrid if map output or laserscan if scan output)
	// if ((map_reg_ == "map") || (map_reg_ == "Map") || (map_reg_ == "MAP")) {
		map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_pub_topic, 1, this);
	// }
	// else if ((map_reg_ == "scan") || (map_reg_ == "Scan") || (map_reg_ == "SCAN")) {
		scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_pub_topic, 1, this);
	// }
	// else {
		// ROS_ERROR_STREAM("[Nav_3d] Post mapper failed to receive the map registration param.  Aborting Post mapper.");
	// }

	if (viz_tool_) {
		viz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(viz_topic, 1, this);
	}

	// Time keeping
	start_time_ = ros::Time::now();
	prev_map_build_time_ = ros::Time(0);

	// Defaulting the robot height. It will be updated to the true value via updateHeight
	current_robot_height_.z = robot_height_default_ + robot_height_buffer_ + robot_height_warning_;
	robot_base_point_.header.frame_id = robot_base_frame_;
	robot_base_point_.header.stamp = ros::Time(0); // Setting to time 0 to allow it to be transformed by tf
	robot_base_point_.point.x = 0;
	robot_base_point_.point.y = 0;
	robot_base_point_.point.z = 0;

	robot_base_point_tfd_.header.frame_id = robot_base_frame_;
	robot_base_point_tfd_.header.stamp = ros::Time(0); // Setting to time 0 to allow it to be transformed by tf
	robot_base_point_tfd_.point.x = 0;
	robot_base_point_tfd_.point.y = 0;
	robot_base_point_tfd_.point.z = 0;

	// Initializing
	poly_init_ = false;
	map_init_ = false;
	scan_init_ = false;
	robot_height_init_ = false;

	ros::Rate loop_rate(loop_rate_);
	ROS_INFO_STREAM("[Nav_3d] PostMapper initialized successfully.");

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void PostMapper::mainCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
	// if ((map_reg_ == "map") || (map_reg_ == "Map") || (map_reg_ == "MAP")) {
		map_to_publish_.header = cloud->header;

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

		prev_map_data_.clear();
		for (int i=0; i<map_to_publish_.data.size(); ++i) {
			prev_map_data_.push_back(map_to_publish_.data[i]);
		}

	// } else if ((map_reg_ == "scan") || (map_reg_ == "Scan") || (map_reg_ == "SCAN")) {
		scan_to_publish_.header = cloud->header;

		// Initializing the scan if the user wants to register the data in scan form
		if (!scan_init_) {
			scan_to_publish_.angle_min = round(-3.141592653589793 * pow(10, scan_res_))/pow(10, scan_res_); // rounding to the scan_res decimal place;
			scan_to_publish_.angle_max = round(3.141592653589793 * pow(10, scan_res_))/(pow(10, scan_res_)); // rounding to the scan_res decimal place;
			scan_to_publish_.angle_increment = pow(10, -scan_res_);
			num_of_pts_ = (int) ((scan_to_publish_.angle_max - scan_to_publish_.angle_min) / scan_to_publish_.angle_increment); //((round(scan_to_publish_.angle_max * pow(10, scan_res_)) / pow(10, scan_res_) - round(scan_to_publish_.angle_min * pow(10, scan_res_)) / pow(10, scan_res_)) / (round(scan_to_publish_.angle_increment * pow(10, scan_res_)) / pow(10, scan_res_)));
			scan_to_publish_.time_increment = 0;  // Unsure of this but it is likely not needed
			scan_to_publish_.scan_time = 0;  // Unsure of this but it is likely not needed as well
			scan_to_publish_.range_min = 0.01;
			scan_to_publish_.range_max = 30.0;
			scan_init_ = true;
			double inf = std::numeric_limits<double>::infinity();

			for (int i=0; i<num_of_pts_; ++i) {
				scan_to_publish_.ranges.push_back(inf);
				scan_to_publish_.intensities.push_back(0);
			}
		}
	// }

	if (!robot_height_init_) {
		ROS_WARN_THROTTLE(30, "The actual robot height has not been initialized.  Currently working off of the robot height default which is set to %f.", robot_height_default_);
	}
	
	// Converting the planar cloud into a XYZ cloud before running any algorithms
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud, *pcl_cloud);

	// Transforming the robot_base_point from the robot_base_frame to the same frame as the cloud
	try {
		listener_.transformPoint(cloud->header.frame_id, robot_base_point_tfd_, robot_base_point_);
	} catch (const tf::TransformException& e) {
		robot_base_point_.header.stamp = ros::Time::now(); // Base point could not be transformed so let's ensure it has the correct time stamp
		ROS_ERROR_THROTTLE(30, "[Nav_3d] Post Mapper could not get the transform from the map frame to the base of the robot.  Slope method will not always output accurate obstacle data is this transform isn't known and neither method will be able to sift out points from the robot polygon. If this error persists you might want to use height method for obstacle detection.");
	}

	// Running a check to see how tilted the lidar is relative to the map.  If it greater than a set amount then slope method will not operate properly so let's set the alg_name to height
	try {
		tf::StampedTransform global_tf, local_tf;
		tf::Quaternion global_rot, local_rot;
		double global_roll, local_roll, pitch, yaw;

		// Looking up the roll from the base_link to the lidar
		listener_.lookupTransform("/base_link", lidar_frame_, ros::Time(0), local_tf);
		local_rot = local_tf.getRotation();
		tf::Matrix3x3 l_mat(local_rot);
		l_mat.getRPY(local_roll, pitch, yaw);

		// Looking up the roll from the map frame to the lidar 
		listener_.lookupTransform("/map", lidar_frame_, ros::Time(0), global_tf);
		global_rot = global_tf.getRotation();
		tf::Matrix3x3 g_mat(global_rot);
		g_mat.getRPY(global_roll, pitch, yaw);


		// Rad to degrees
		global_roll = global_roll * 180 / 3.1416;
		local_roll = local_roll * 180 / 3.1416;

		// If the lidar is tilted relative to the map vs the base_link more than the threshold in degrees than we will always use height method
		if (fabs(global_roll - local_roll) > lidar_tilt_threshold_){
			alg_name_ = "height";
		}

	} catch (const tf::TransformException& e) {
		ROS_ERROR_THROTTLE(30, "[Nav_3d] Post mapper could not get the transform from the map frame to the lidar frame. If the lidar is significantly rolled relative to the map frame slope method may not perform properly.");
	}

	// Calling the right algorithm to run on the cloud to find the obstacles
	if (alg_name_ == "height" || alg_name_ == "Height" || alg_name_ == "HEIGHT" || alg_name_ == "height_method" || alg_name_ == "height method" || alg_name_ == "HEIGHT METHOD" || alg_name_ == "Height Method") {
		points_checked_.clear();

		// If the polygon and the points being read are different frames we'll transform the polygon
		if (current_poly_.header.frame_id != cloud->header.frame_id) {
			this->convertPoly(cloud->header);
		}

		this->heightMethod(pcl_cloud);

		// Since we have the new size of the map we can assign proper cell values to each point in cloud
		this->mapDiscretizer();

	} else if (alg_name_ == "slope" || alg_name_ == "Slope" || alg_name_ == "SLOPE" || alg_name_ == "slope_method" || alg_name_ == "slope method" || alg_name_ == "SLOPE METHOD" || alg_name_ == "Slope Method") {

		// If the polygon and the points being read are different frames we'll transform the polygon
		if (current_poly_.header.frame_id != robot_base_frame_) {
			std_msgs::Header header;
			header.frame_id = robot_base_frame_;
			header.stamp = ros::Time::now();
			this->convertPoly(header);
		}

		// Before slope method we have to parse the raw cloud understanding the lidar config
		this->cloudParser(pcl_cloud);

		// Since we have the new size of the map we can assign proper cell values to each point in cloud
		this->mapDiscretizer();

		points_checked_.clear();
		this->slopeMethod();
	} else {
		ROS_ERROR_STREAM("[Nav_3d] Failed to receive an algorithm to run.  Cannot perform obstacle detection. Check the yaml algorithm entry to ensure it is typed correctly.");
	}

	// Build the map now
	// if ((map_reg_ == "map") || (map_reg_ == "Map") || (map_reg_ == "MAP")) {
		this->mapBuilder();
	// } else if ((map_reg_ == "scan") || (map_reg_ == "Scan") || (map_reg_ == "SCAN")) {
		this->scanBuilder();
	// }

	// Publish
	// if ((map_reg_ == "map") || (map_reg_ == "Map") || (map_reg_ == "MAP")) {
		map_pub_.publish(map_to_publish_);
	// } else if ((map_reg_ == "scan") || (map_reg_ == "Scan") || (map_reg_ == "SCAN")) {
		scan_pub_.publish(scan_to_publish_);
	// }


	// Visualize data in point cloud form if viz tool is set to active
	if (viz_tool_) {
		this->visualizationTool();
	}
}

void PostMapper::heightMethod(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
	bool in_poly;
	geometry_msgs::PointStamped cloud_point;
	float local_distance;
	cloud_point.header.frame_id = cloud->header.frame_id;

	point_XYZDATO checked_point;
	for (int i=0; i<cloud->points.size(); ++i) {
		if (cloud->points[i].z < current_robot_height_.z && (sqrt(cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y) > min_obj_dist_) && cloud->points[i].z > -max_ditch_depth_) {
			this->mapSizeMaintainer(cloud->points[i]);
			
			checked_point.x = cloud->points[i].x;
			checked_point.y = cloud->points[i].y;
			checked_point.z = cloud->points[i].z - robot_base_point_.point.z;
			checked_point.distance = sqrt(checked_point.x * checked_point.x + checked_point.y * checked_point.y);
			checked_point.time_stamp = ros::Time::now();
			checked_point.obstacle = 0;

	        // Check to see if the point is within the robot footprint
	        // This is made efficient by first checking to make sure the point is within the robot possible max reach
			local_distance = sqrt((checked_point.x - robot_base_point_.point.x) * (checked_point.x - robot_base_point_.point.x) + (checked_point.y - robot_base_point_.point.y) * (checked_point.y - robot_base_point_.point.y));
	        if (poly_init_ && local_distance < max_robot_reach_) {	
	            cloud_point.point.x = checked_point.x;
	            cloud_point.point.y = checked_point.y;
	            
	            in_poly = point_in_poly(current_poly_, cloud_point);
	        } else {
	            // Even if the polygon is not initialized 
	            in_poly = false;
	        }

	        if (!in_poly) {
				// This is how the height method determines what the ground is
				if (!(cloud->points[i].z > - floor_range_ && cloud->points[i].z <  floor_range_)) {			
						
					// If the height is in the warning zone we will label it as such
					if (checked_point.z > current_robot_height_.z - robot_height_warning_) {
						checked_point.obstacle = 4;
					} else {
						checked_point.obstacle = 1;
					}

					occupied_list_.push_back(checked_point);
				}
				points_checked_.push_back(checked_point);
	        }
		}
	}
}

void PostMapper::slopeMethod() {
	bool in_poly;
	geometry_msgs::PointStamped cloud_point;

	// Since cloud parser translated front and back cloud to robot_base_frame the cloud point we need to check is in that same frame and not the cloud frame
	cloud_point.header.frame_id = robot_base_frame_;

	// Need to create a local checked point variable which will be in the map reference frame and sent to the map builder
	point_XYZDATO checked_point;	

	// Perform the slope method calculation on the front cloud
	// NOTE it is important that the first assumed ground point is at 0
	// and we begin the first point to analyze at 1
	float slope;
	int g;
	for (int k=0; k<parsed_clouds_.size(); ++k) {

		g = 0;
		for (int i=1; i<parsed_clouds_[k].size(); ++i) {
			// /Ignore all points that are higher than the highest point in the robot
			if (parsed_clouds_[k][i].z < current_robot_height_.z && parsed_clouds_[k][i].z > -max_ditch_depth_ && parsed_clouds_[k][i].distance > min_obj_dist_) {

	            // Check to see if the point is within the robot footprint
	            // This is made efficient by first checking to make sure the point is within the robot possible max reach
	            if (poly_init_ && parsed_clouds_[k][i].distance < max_robot_reach_) {	
	                cloud_point.point.x = parsed_clouds_[k][i].x;
	                cloud_point.point.y = parsed_clouds_[k][i].y;
	                
	                in_poly = point_in_poly(current_poly_, cloud_point);
	            } else {
	                // Even if the polygon is not initialized 
	                in_poly = false;
	            }

	            // If the point is not inside the polygon then we are going to check to see if it is an obstacle point
	            if ((!in_poly) && ((parsed_clouds_[k][i].distance - parsed_clouds_[k][g].distance) < max_check_dist_)) {

	                // Slope calculation and check
	                slope = (parsed_clouds_[k][i].z - parsed_clouds_[k][g].z) / (parsed_clouds_[k][i].distance - parsed_clouds_[k][g].distance);
	                if (fabs(slope) > slope_threshold_) {

	                    // If the point is flagged by having a sharp slope it can still be in the likely drivable region if it is within the drivable height
	                    if (fabs(parsed_clouds_[k][i].z - parsed_clouds_[k][g].z) > drivable_height_) {
							
							// If the height is in the warning zone and the point is defintely an obstacle we will label it as such
							if(parsed_clouds_[k][i].z > current_robot_height_.z - robot_height_warning_) {
								parsed_clouds_[k][i].obstacle = 4;
							} else {
								parsed_clouds_[k][i].obstacle = 1;
							}

	                    } else {
	                    	// Likely drivable point
	                    	parsed_clouds_[k][i].obstacle = 2;
	                    }

	                // If the point has passed the slope test but is at a great height relative to the last ground point then we aren't sure it is the ground
	                } else if (fabs(parsed_clouds_[k][i].z - parsed_clouds_[k][g].z) > max_step_height_){
	                    parsed_clouds_[k][i].obstacle = 3;

	                // If the point passes the slope and the height tests then the new furthest ground point is at the current index
	                } else {
	                    g = i;
	                }
	                // Accumulating a vector of all the points that have been checked in this callback
	                // Need to translate point back into the map frame first
					checked_point.x = parsed_clouds_[k][i].x + robot_base_point_.point.x;
					checked_point.y = parsed_clouds_[k][i].y + robot_base_point_.point.y;
					checked_point.z = parsed_clouds_[k][i].z + robot_base_point_.point.z;
					checked_point.distance = sqrt(checked_point.x * checked_point.x + checked_point.y * checked_point.y);
					checked_point.obstacle = parsed_clouds_[k][i].obstacle;
					checked_point.cloud_seg = parsed_clouds_[k][i].cloud_seg;
					checked_point.time_stamp = ros::Time::now();
	                points_checked_.push_back(checked_point);
	                
	                if(checked_point.obstacle == 1) {
	                	occupied_list_.push_back(checked_point);
	                }

	            }
			}
		}
	}
}

void PostMapper::cloudParser(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
	parsed_clouds_.clear();
	std::vector<point_XYZDATO> cloud_seg;
	
	for (int i=0; i<parsing_segments_; ++i) {
		parsed_clouds_.push_back(cloud_seg);
	}

	point_XYZDATO cloud_point;
	float angle_index = 360 / parsing_segments_; 
	int k;

	// Building the front half of the cloud for analysis
	for (int i=0; i<cloud->points.size(); ++i) {
		this->mapSizeMaintainer(cloud->points[i]);

		// Need to subtract the xyz coordinates of the robot_base_frame to tranlate it into the robot_base_frame
		cloud_point.x = cloud->points[i].x - robot_base_point_.point.x;
		cloud_point.y = cloud->points[i].y - robot_base_point_.point.y;
		cloud_point.z = cloud->points[i].z - robot_base_point_.point.z;
		cloud_point.distance = sqrt(cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y);
		cloud_point.azimuth = 180 + (180 / 3.1416) * std::atan2(cloud->points[i].y, cloud->points[i].x);
		cloud_point.time_stamp = ros::Time::now();
		cloud_point.obstacle = 0;

		k = floor(cloud_point.azimuth / angle_index);
		// Tagging the the point with which segment it is in for vizualization
		cloud_point.cloud_seg = k;

		parsed_clouds_[k].push_back(cloud_point);
	}

	// Adding an all zeros robot base point and then sorting the all the clouds by distance
	point_XYZDATO zero_base_point;
	zero_base_point.x = 0;
	zero_base_point.y = 0;
	zero_base_point.z = 0;
	zero_base_point.distance = 0;
	zero_base_point.azimuth = 0;
	zero_base_point.time_stamp = ros::Time::now();

	for (int i=0; i<parsed_clouds_.size(); ++i) {
		parsed_clouds_[i].push_back(zero_base_point);
		std::sort(parsed_clouds_[i].begin(), parsed_clouds_[i].end(), compareDistance);
	}
}

// The Map Builder function contructs an OccupancyGrid map based on the class varibale occupied_list and new_obs.
// New_obs is strictly the new vector of obstacle points that have been added in this most recent spin
void PostMapper::mapBuilder() {
	bool stale_map;
	if(prev_map_build_time_ == ros::Time(0)) {
		stale_map = true;
	} else if(ros::Duration(ros::Time::now() - prev_map_build_time_).toSec() > stale_map_time_) {
		stale_map = true;
	} else {
		stale_map = false;
	}

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

				if(prev_map_data_[i] <= 0) {
					map_to_publish_.data[new_index] = prev_map_data_[i];
				} else {
					map_to_publish_.data[new_index] = prev_map_data_[i];
				}
			}
        // If the map only changed size in the positive y direction then we can just add on unknown points to the end of the data
		} else {
			while (map_to_publish_.data.size() < new_map_size) {
				map_to_publish_.data.push_back(-1);
			}
		}
	}

	int y_placement, x_placement, placement_spot;

	// The list of cells to recently be declared as an obstacle must be cleared if the data is old or 
	if (map_dim_change_ || (ros::Duration(ros::Time::now() - prev_recent_obs_purge_time_).toSec() > recent_obs_purge_time_)) {
		recent_obs_cells_.clear();
		prev_recent_obs_purge_time_ = ros::Time::now();
	}

	// Running throught the new points that have been checked
	for (int i=0; i<points_checked_.size(); ++i) {

		// Only going to build in points that are definitely not an obstacle (0) or definitely an obstacle (1) into the map
		if (points_checked_[i].obstacle < 3) {

			// Placement spots to determine where to put the data in the map
			x_placement = (int) round((points_checked_[i].x - map_to_publish_.info.origin.position.x) / map_res_);
			y_placement = (int) round((points_checked_[i].y - map_to_publish_.info.origin.position.y) / map_res_);
			placement_spot = (y_placement) * map_to_publish_.info.width + x_placement;

			// If the cell has been labeled as a recent obstacle then we will no longer process it for this cycle
			if ( (std::find(recent_obs_cells_.begin(), recent_obs_cells_.end(), placement_spot) == recent_obs_cells_.end()) ) {

				// If the point checked is a new obstacle then set it to 99 on the map
				if (points_checked_[i].obstacle == 1) {//&& map_to_publish_.data[placement_spot] <= 0) {
					map_to_publish_.data[placement_spot] = 99;
					recent_obs_cells_.push_back(placement_spot);
				// The post mapper will check against the live mapper. If the live mapper had some probability of obstacle then lets increase it but not set at 99
		        // } else if (points_checked_[i].obstacle == 1 && map_to_publish_.data[placement_spot] > 0) {
		        // 	map_to_publish_.data[placement_spot] = map_to_publish_.data[placement_spot] * (obs_decay_factor_ + 1);
		        // 	if (map_to_publish_.data[placement_spot] > 99) {
		        // 		map_to_publish_.data[placement_spot] = 99;
		        // 	}

		        // If the point checked is previously unknown and the point is labeled as drivable or likely drivable as an obstacle then it will be assigned as a ground point
				} else if (map_to_publish_.data[placement_spot] <= 0 && ((points_checked_[i].obstacle == 0)  || (points_checked_[i].obstacle == 2)) ) {
					map_to_publish_.data[placement_spot] = 0;
		        // do the obstacle decay if the point has been detected as drivable or likely drivable in a previously scanned cell
				} else if ( (points_checked_[i].obstacle == 0)  || (points_checked_[i].obstacle == 2) ) {

					// If the point is a ground or likely drivable point then reduce obstacle probability in cell
					map_to_publish_.data[placement_spot] = map_to_publish_.data[placement_spot] * (1 - obs_decay_factor_);
				}
			}
		}
	}

	prev_map_build_time_ = ros::Time::now();
}

// The Scan Builder function contructs a faux 2D laserscan based on the class varibale occupied_list.
void PostMapper::scanBuilder() {
	int obs_prob, placement_spot;

	for (int i=0; i<occupied_list_.size(); ++i) {
		if (obs_decay_type_ == "exponential" || obs_decay_type_ == "exp" || obs_decay_type_ == "Exponential" || obs_decay_type_ == "EXP") {
			obs_prob = std::min((int)(99 * exp(-(ros::Duration(ros::Time::now() - occupied_list_[i].time_stamp).toSec()) / obs_decay_time_ )) , 99);
			// Modeled like a decay function N0 * e ^ -t / Tau where Tau (obs_decay_time) is half life			
		} else if (obs_decay_type_ == "linear" || obs_decay_type_ == "lin" || obs_decay_type_ == "Linear" || obs_decay_type_ == "LIN") {
			obs_prob = std::min((int)(99 - 99 * (ros::Duration(ros::Time::now() - occupied_list_[i].time_stamp).toSec()) / obs_decay_time_ ) , 99);
			// Modeled linearly where obs_decay_time is the time for the obstacle to reach 0 probability			
		}

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

void PostMapper::mapSizeMaintainer(const pcl::PointXYZ point) {
	if(point.x > max_x_) {
		max_x_ = point.x + 5 *map_res_;
	}

	if(point.y > max_y_) {
		max_y_ = point.y + 5 *map_res_;
	}

	if(point.x < min_x_) {
		min_x_ = point.x - 5 *map_res_;
	}

	if(point.y < min_y_){
		min_y_ = point.y - 5 *map_res_;
	}
}

void PostMapper::mapDiscretizer() {
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

	// Checking to see if the size of the map has changed.  No need to refill an entire new map if it is the same size
	map_dim_change_ = (prev_width_ != map_to_publish_.info.width) || (prev_height_ != map_to_publish_.info.height);
}

void PostMapper::convertPoly(const std_msgs::Header new_header) {
	geometry_msgs::PointStamped temp_poly_point, temp_poly_point_tfd;
	bool tf_fail = false;

	for (int i=0; i<current_poly_.polygon.points.size(); ++i) {
		temp_poly_point.header = temp_poly_point_tfd.header = current_poly_.header;
		temp_poly_point.point.x = temp_poly_point_tfd.point.x = current_poly_.polygon.points[i].x;
		temp_poly_point.point.y = temp_poly_point_tfd.point.y = current_poly_.polygon.points[i].y;
		temp_poly_point.point.z = temp_poly_point_tfd.point.z = current_poly_.polygon.points[i].z;

		try {
			listener_.transformPoint(new_header.frame_id, temp_poly_point_tfd, temp_poly_point);

			current_poly_.polygon.points[i].x = temp_poly_point.point.x;
			current_poly_.polygon.points[i].y = temp_poly_point.point.y;
			current_poly_.polygon.points[i].z = temp_poly_point.point.z;
			
		} catch (const tf::TransformException& e) {
			ROS_ERROR_THROTTLE(30, "[Nav_3d] Post Mapper could not convert the robot polygon into the same frame as the incoming cloud scan.  It is likely that obstacle data around the robot will be inaccurate.");
			tf_fail = true;
		}

	}

	if (!(current_poly_.header.frame_id == new_header.frame_id) && !tf_fail) {
		current_poly_.header.frame_id = new_header.frame_id;
	}

}

void PostMapper::updatePoly(const geometry_msgs::PolygonStamped::ConstPtr& new_poly_stamped) {
	current_poly_ = *new_poly_stamped;
	poly_init_ = true;
}

void PostMapper::updateHeight(const geometry_msgs::Point32::ConstPtr& new_height) {
	current_robot_height_ = *new_height;
	robot_height_init_ = true;
}

void PostMapper::updateMap(const nav_msgs::OccupancyGrid::ConstPtr& new_map) {
	map_to_publish_ = *new_map;
	
	for (int i=0; i<map_to_publish_.data.size(); ++i) {
		prev_map_data_.push_back(map_to_publish_.data[i]);
	}
	
	map_init_ = true;
}


void PostMapper::visualizationTool() {
	pcl::PointXYZI point;

	// Timer or rotation check
	if (prev_viz_pub_time_ == ros::Time(0)) {
		
		viz_cloud_.height = 1;
		viz_cloud_.header.frame_id = "map";

		// Initiating a point with the highest intesity value to maintain the color scale
		point.x = 0;
		point.y = 0;
		point.z = 0;
		point.intensity = 4;
		++viz_cloud_.width;
		viz_cloud_.points.push_back(point);

	} else if (ros::Duration(ros::Time::now() - prev_viz_pub_time_).toSec() > viz_timer_) {

		viz_cloud_.width = 0;
		viz_cloud_.points.clear();

		// Initiating a point with the highest intesity value to maintain the color scale
		point.x = 0;
		point.y = 0;
		point.z = 0;
		point.intensity = 4;
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
		//point.intensity = points_checked_[i].obstacle;
		point.intensity = points_checked_[i].obstacle;
		
		viz_cloud_.points.push_back(point);
	}

	pcl::toROSMsg(viz_cloud_, viz_cloud_pub_);
	viz_pub_.publish(viz_cloud_pub_);

	prev_viz_pub_time_ = ros::Time::now();
}


bool PostMapper::compareDistance(point_XYZDATO p1, point_XYZDATO p2) {
	return (p1.distance < p2.distance);
}

int main(int argc, char** argv) {
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	ros::init(argc, argv, "nav_3d/post_mapper");

	PostMapper post_mapper;
}
