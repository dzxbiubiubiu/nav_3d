
#include "live_mapper.h"

/* ------------------------- Constructor -------------------------
  Attempts to initialize most variables from ros Parameter Server
    Defaults are set for most things, in case yaml isn't loaded\
*/

LiveMapper::LiveMapper()
{
	// These variables not used elsewhere in class...
	std::string planar_cloud_topic, poly_topic, height_topic, map_pub_topic;

	if(!nh_.param<std::string>("nav_3d/map_pub_topic", map_pub_topic, "nav_3d/live_map"))
		ROS_WARN_STREAM("[Nav_3d] Failed to get map pub topic from parameter server - defaulting to " << map_pub_topic << ".");
	if(!nh_.param<std::string>("nav_3d/planar_cloud_topic", planar_cloud_topic, "laser_stitcher/planar_cloud") )
		ROS_WARN_STREAM("[Nav_3d] Failed to get planar cloud topic from parameter server - defaulting to " << planar_cloud_topic << ".");
	nh_.param<std::string>("nav_3d/obstacle_algorithm", alg_name_, "slope");
	nh_.param<std::string>("nav_3d/poly_topic", poly_topic, "nav_3d/robot_footprint");
	nh_.param<std::string>("nav_3d/height_topic", height_topic, "nav_3d/robot_height");
	nh_.param<float>("nav_3d/robot_height_default", robot_height_default_, 1);
	nh_.param<float>("nav_3d/floor_range", floor_range_, 0.1);
	nh_.param<float>("nav_3d/min_obj_dist", min_obj_dist_, 0.0);
	nh_.param<int>("nav_3d/res", res_, 3);
	nh_.param<float>("nav_3d/map_res", map_res_, 0.05);
	nh_.param<float>("nav_3d/slope_threshold", slope_threshold_, 3);
	nh_.param<float>("nav_3d/drivable_height", drivable_height_, 0.1);
	nh_.param<float>("nav_3d/stale_map_time", stale_map_time_, 10);
	nh_.param<float>("nav_3d/obs_decay_time", obs_decay_time_, 90);
	nh_.param<float>("nav_3d/pub_rate", pub_rate_, 100);

	// Subscribers
	planar_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(planar_cloud_topic, 1, &LiveMapper::mapPublisher, this);
	poly_sub_ = nh_.subscribe<geometry_msgs::PolygonStamped>(poly_topic, 1, &LiveMapper::updatePoly, this);
	height_sub_ = nh_.subscribe<geometry_msgs::Point32>(height_topic, 1, &LiveMapper::updateHeight, this);

	// Publisher
	map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_pub_topic, 1, this);
	
	// Time keeping
	start_time_ = ros::Time::now();
	prev_map_build_time_ = ros::Time(0);
	// prev_map_build_time_ = ros::Time(0) - ros::Duration(stale_map_time_ + 1);

	// Defaulting the robot height. It will be updated to the true value via updateHeight
	current_robot_height_.z = robot_height_default_;

	while(ros::ok())
		ros::spinOnce();
}

void LiveMapper::mapPublisher(const sensor_msgs::PointCloud2::ConstPtr& planar_cloud)
{
	map_to_publish_.header = planar_cloud->header;

	// Initializing the map
	if(!map_init_)
	{
		map_to_publish_.info.resolution = map_res_;
		map_to_publish_.info.origin.position.z = 0;
		map_to_publish_.info.origin.orientation.x = 0;
		map_to_publish_.info.origin.orientation.y = 0;
		map_to_publish_.info.origin.orientation.z = 0;
		map_to_publish_.info.origin.orientation.w = 1.0;
		map_init_ = true;
	}

	//If the polygon and the points being read are different frames we'll transform the polygon
	if(current_poly_.header.frame_id != planar_cloud->header.frame_id)
	{
		geometry_msgs::PointStamped temp_poly_point;

		for(int i=0; i<current_poly_.polygon.points.size(); i++)
		{
			temp_poly_point.header = current_poly_.header;
			temp_poly_point.point.x = current_poly_.polygon.points[i].x;
			temp_poly_point.point.y = current_poly_.polygon.points[i].y;
			temp_poly_point.point.z = current_poly_.polygon.points[i].z;

			listener_.transformPoint(planar_cloud->header.frame_id, temp_poly_point, temp_poly_point);
			current_poly_.polygon.points[i].x = temp_poly_point.point.x;
			current_poly_.polygon.points[i].y = temp_poly_point.point.y;
			current_poly_.polygon.points[i].z = temp_poly_point.point.z;
		}

		current_poly_.header.frame_id = planar_cloud->header.frame_id;
	}

	if(!robot_height_init_)
		ROS_WARN_THROTTLE(30, "The actual robot height has not been initialized.  Currently working off of the robot height default which is set to %f.", robot_height_default_);

	if(alg_name_ == "height" || alg_name_ == "Height" || alg_name_ == "HEIGHT" || alg_name_ == "height_method" || alg_name_ == "height method" || alg_name_ == "HEIGHT METHOD" || alg_name_ == "Height Method")
		this->heightMethod(planar_cloud);
	else if(alg_name_ == "slope" || alg_name_ == "Slope" || alg_name_ == "SLOPE" || alg_name_ == "slope_method" || alg_name_ == "slope method" || alg_name_ == "SLOPE METHOD" || alg_name_ == "Slope Method")
		this->slopeMethod(planar_cloud);
	else
		ROS_ERROR_STREAM("[Nav_3d] Failed to receive an algorithm to run.  Cannot perform obstacle detection.");

	map_pub_.publish(map_to_publish_);
}

void LiveMapper::heightMethod(const sensor_msgs::PointCloud2::ConstPtr& planar_cloud)
{
	geometry_msgs::PointStamped cloud_point;
	std::vector<point_XYZDTC> new_obs_;
	cloud_point.header = planar_cloud->header;
	
	// Converting the planar cloud into a XYZ cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*planar_cloud, *temp_cloud);

	point_XYZDTC temp_obs_point;
	for(int i=0; i<temp_cloud->points.size(); i++)
	{
		ROS_INFO_STREAM(current_robot_height_.z);
		if(temp_cloud->points[i].z < current_robot_height_.z)
		{
			if(temp_cloud->points[i].x > max_x_)
				max_x_ = temp_cloud->points[i].x;
			if(temp_cloud->points[i].y > max_y_)
				max_y_ = temp_cloud->points[i].y;
			if(temp_cloud->points[i].x < min_x_)
				min_x_ = temp_cloud->points[i].x;
			if(temp_cloud->points[i].y < min_y_)
				min_y_ = temp_cloud->points[i].y;

			// This is how the height method determines what the ground is
			if(!(temp_cloud->points[i].z > - floor_range_ && temp_cloud->points[i].z < floor_range_))
			{
				if(poly_init_)
				{
					cloud_point.point.x = temp_cloud->points[i].x;
					cloud_point.point.y = temp_cloud->points[i].y;

					temp_obs_point.x = temp_cloud->points[i].x;
					temp_obs_point.y = temp_cloud->points[i].y;
					temp_obs_point.z = temp_cloud->points[i].z;
					temp_obs_point.distance = sqrt(temp_obs_point.x * temp_obs_point.x + temp_obs_point.y * temp_obs_point.y);
					temp_obs_point.time_stamp = ros::Time::now();
					temp_obs_point.count_stamp = i;

					// Check to see if the points is within the robot
					// in_poly = point_in_poly(current_poly_, cloud_point);

					// If the point is not in the polygon then we build it in as an obstacle					

					// if(!in_poly)
					// {
						occupied_list_.push_back(temp_obs_point);
						new_obs_.push_back(temp_obs_point);
					// }
				}
				else
				{
					occupied_list_.push_back(temp_obs_point);
					new_obs_.push_back(temp_obs_point);					
				}
			}
		}
	}

	this->mapBuilder(new_obs_);
}

void LiveMapper::slopeMethod(const sensor_msgs::PointCloud2::ConstPtr& planar_cloud)
{
	geometry_msgs::PointStamped cloud_point;
	std::vector<point_XYZDTC> new_obs_;
	cloud_point.header = planar_cloud->header;
	
	// Converting the planar cloud into a XYZ cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*planar_cloud, *temp_cloud);

	int mid_index;
	// If the size of the point vector is odd we will throw out the middle point
	if(temp_cloud->width % 2 != 0)
	{
		mid_index = (temp_cloud->width - 1) / 2;
		temp_cloud->points.erase(temp_cloud->points.begin() + mid_index);
	}
	// If it is even then we will just capture the mid_index for use later
	else
		mid_index = temp_cloud->width / 2;

	int cloud_size = temp_cloud->points.size();

	// Building the front half of the cloud for analysis
	std::vector<point_XYZDTC> front_cloud(cloud_size / 2);
	for(int i=0; i<mid_index; i++)
	{
		front_cloud[i].x = temp_cloud->points[i].x;
		front_cloud[i].y = temp_cloud->points[i].y;
		front_cloud[i].z = temp_cloud->points[i].z;
		front_cloud[i].distance = sqrt(front_cloud[i].x * front_cloud[i].x + front_cloud[i].y * front_cloud[i].y);		
		front_cloud[i].time_stamp = ros::Time::now();
		front_cloud[i].count_stamp = i; // counter for sanity checks
	}

	// Adding an all zeros robot base point and then sorting the front cloud by distance
	point_XYZDTC robot_base_point;
	front_cloud.push_back(robot_base_point);
	std::sort(front_cloud.begin(), front_cloud.end(), compareDistance);

	// Building the back half of the cloud for analysis
	std::vector<point_XYZDTC> back_cloud(cloud_size / 2);
	for(int i=0; i<mid_index; i++)
	{
		back_cloud[i].distance = sqrt(temp_cloud->points[i + mid_index].x * temp_cloud->points[i + mid_index].x + temp_cloud->points[i + mid_index].y * temp_cloud->points[i + mid_index].y);
		back_cloud[i].x = temp_cloud->points[i + mid_index].x;
		back_cloud[i].y = temp_cloud->points[i + mid_index].y;
		back_cloud[i].z = temp_cloud->points[i + mid_index].z;
		back_cloud[i].time_stamp = ros::Time::now();
		back_cloud[i].count_stamp = i; // counter for sanity checks
	}

	// Adding an all zeros robot base point and then sorting the back cloud by distance
	front_cloud.push_back(robot_base_point);
	std::sort(back_cloud.begin(), back_cloud.end(), compareDistance);

	// Perform the slope method calculation on the front cloud
	// NOTE it is important that the known ground point is at 0
	// and we begin the first point to analyze at 1
	int g = 0;
	for(int i=1; i<front_cloud.size(); i++)
	{
		if(front_cloud[i].z < current_robot_height_.z && front_cloud[i].distance > min_obj_dist_)
		{
			if(front_cloud[i].x > max_x_)
				max_x_ = front_cloud[i].x;
			if(front_cloud[i].y > max_y_)
				max_y_ = front_cloud[i].y;
			if(front_cloud[i].x < min_x_)
				min_x_ = front_cloud[i].x;
			if(front_cloud[i].y < min_y_)
				min_y_ = front_cloud[i].y;

			float slope = (front_cloud[i].z - front_cloud[g].z) / (front_cloud[i].distance - front_cloud[g].distance);

			if(fabs(slope) > slope_threshold_)
			{
				if(poly_init_)
				{
					cloud_point.point.x = front_cloud[i].x;
					cloud_point.point.y = front_cloud[i].y;

					// Check to see if the point is within the robot footprint
					// in_poly = point_in_poly(current_poly_, cloud_point);

					// if(!in_poly && fabs(front_cloud[i].z - front_cloud[g].z) > drivable_height_)
					// {
						occupied_list_.push_back(front_cloud[i]);
						new_obs_.push_back(front_cloud[i]);
					// }
				}
				else
				{
					if(fabs(front_cloud[i].z - front_cloud[g].z) > drivable_height_)
					{
						occupied_list_.push_back(front_cloud[i]);
						new_obs_.push_back(front_cloud[i]);
					}
				}
			}
			else
			{
				// The new furthest ground point is at the current index
				g = i;
			}
		}
	}

	// Perform the slope method calculation on the back cloud
	// NOTE it is important that the known ground point is at 0
	// and we begin the first point to analyze at 1
	g = 0;
	for(int i=1; i<back_cloud.size(); i++)
	{
		if(back_cloud[i].z < current_robot_height_.z && back_cloud[i].distance > min_obj_dist_)
		{
			if(back_cloud[i].x > max_x_)
				max_x_ = back_cloud[i].x;
			if(back_cloud[i].y > max_y_)
				max_y_ = back_cloud[i].y;
			if(back_cloud[i].x < min_x_)
				min_x_ = back_cloud[i].x;
			if(back_cloud[i].y < min_y_)
				min_y_ = back_cloud[i].y;

			float slope = (back_cloud[i].z - back_cloud[g].z) / (back_cloud[i].distance - back_cloud[g].distance);

			if(fabs(slope) > slope_threshold_)
			{
				if(poly_init_)
				{
					cloud_point.point.x = back_cloud[i].x;
					cloud_point.point.y = back_cloud[i].y;

					// Check to see if the point is within the robot footprint
					// in_poly = point_in_poly(current_poly_, cloud_point);

					// if(!in_poly && fabs(back_cloud[i].z - back_cloud[g].z) > drivable_height_)
					// {
						occupied_list_.push_back(back_cloud[i]);
						new_obs_.push_back(back_cloud[i]);
					// }
				}
				else
				{
					if(fabs(back_cloud[i].z - back_cloud[g].z) > drivable_height_)
					{
						occupied_list_.push_back(back_cloud[i]);
						new_obs_.push_back(back_cloud[i]);
					}
				}
			}
			else
			{
				// The new furthest ground point is at the current index
				g = i;
			}
		}
	}

	this->mapBuilder(new_obs_);
}

// The Map Builder function contructs an OccupancyGrid map based on the class varibale occupied_list and new_obs.
// New_obs is strictly the new vecotr of obstacle points that have been added in this most recent spin
void LiveMapper::mapBuilder(const std::vector<point_XYZDTC> new_obs_)
{
	bool stale_map;
	if(prev_map_build_time_ == ros::Time(0))
		stale_map = true;
	else if(ros::Duration(ros::Time::now() - prev_map_build_time_).toSec() > stale_map_time_)
		stale_map = true;
	else
		stale_map = false;

	int prev_height = map_to_publish_.info.height;
	int prev_width = map_to_publish_.info.width;

	map_to_publish_.info.origin.position.x = round(min_x_*10)/10; // rounding to the 2nd decimal place
	map_to_publish_.info.origin.position.y = round(min_y_*10)/10; // rounding to the 2nd decimal place
	map_to_publish_.info.width = (int)((max_x_ - min_x_) / map_res_);
	map_to_publish_.info.height = (int)((max_y_ - min_y_) / map_res_);

	// Checking to see if the size of the map has changed.  No need to refill an entire new map if it is the same size
	bool map_dim_change = (prev_width != map_to_publish_.info.width) || (prev_height != map_to_publish_.info.height);
	if(map_dim_change)
	{
		int map_size = map_to_publish_.info.width * map_to_publish_.info.height;
		for(int i=0; i<map_to_publish_.data.size(); i++)
		{
			map_to_publish_.data[i] = 0;
		}
		while(map_to_publish_.data.size() < map_size)
		{
			map_to_publish_.data.push_back(0);
		}
	}

	int obs_prob, y_placement, x_placement, placement_spot;
	if(map_dim_change || stale_map)
	{
		for(int i=0; i<occupied_list_.size(); i++)
		{
			obs_prob = std::min((int)(99 * exp(-(ros::Duration(ros::Time::now() - occupied_list_[i].time_stamp).toSec()) / obs_decay_time_ )) , 99);
			// Modeled like a decay function N0 * e ^ -t / Tau where Tau (obs_decay_time) is half life
			// obs_prob = std::min((int)(99 - 99 * (ros::Duration(ros::Time::now() - occupied_list_[i].time_stamp).toSec()) / obs_decay_time_ ) , 99);
			// Modeled linearly where obs_decay_time is the time for the obstacle to reach 0 probability
			
			// If the probability of an obstacle is greater than 5% we will build it into the map
			if(obs_prob > 5)
			{
				if(!(occupied_list_[i].x < min_x_ || occupied_list_[i].x > max_x_ || occupied_list_[i].y < min_y_ || occupied_list_[i].y > max_y_))
				{
					x_placement = (int)((occupied_list_[i].x - map_to_publish_.info.origin.position.x) / map_res_);
					y_placement = (int)((occupied_list_[i].y - map_to_publish_.info.origin.position.y) / map_res_);
					placement_spot = (y_placement) * map_to_publish_.info.width + x_placement;

					if(placement_spot < map_to_publish_.data.size())
						map_to_publish_.data[placement_spot] = obs_prob;
					else
						occupied_list_.erase(occupied_list_.begin() + i);
				}
			}
		}
	}
	// If map dimensions did not change we can be smart and only place the obstacles that are new
	else
	{
		for(int i=0; i<new_obs_.size(); i++)
		{
			if(!(new_obs_[i].x < min_x_ || new_obs_[i].x > max_x_ || new_obs_[i].y < min_y_ || new_obs_[i].y > max_y_))
			{
				x_placement = (int)((new_obs_[i].x - map_to_publish_.info.origin.position.x) / map_res_);
				y_placement = (int)((new_obs_[i].y - map_to_publish_.info.origin.position.y) / map_res_);
				placement_spot = (y_placement) * map_to_publish_.info.width + x_placement;

				if(placement_spot < map_to_publish_.data.size())
					map_to_publish_.data[placement_spot] = 99; // If it is a new obstacle we will give it a high probability
			}
		}
	}

	prev_map_build_time_ = ros::Time::now();
}

void LiveMapper::updatePoly(const geometry_msgs::PolygonStamped::ConstPtr& new_poly_stamped)
{
	current_poly_ = *new_poly_stamped;
	poly_init_ = true;
}

void LiveMapper::updateHeight(const geometry_msgs::Point32::ConstPtr& new_height)
{
	current_robot_height_ = *new_height;
	robot_height_init_ = true;
}

bool LiveMapper::compareDistance(point_XYZDTC p1, point_XYZDTC p2)
{
	return (p1.distance < p2.distance);
}

int main(int argc, char** argv)
{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);	
	ros::init(argc, argv, "live_mapper");

	LiveMapper live_mapper;

	// ros::spin();
	
	// return(0);
}