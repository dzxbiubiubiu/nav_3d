
#ifndef LIVE_MAPPER_H
#define LIVE_MAPPER_H

#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include "point_in_poly.h"

#include <tf/transform_listener.h>

class LiveMapper
{
public:
	LiveMapper();
private:
	ros::NodeHandle nh_;
	ros::Subscriber planar_cloud_sub_;
	ros::Subscriber poly_sub_;
	ros::Subscriber height_sub_;
	ros::Publisher map_pub_;
	ros::Publisher viz_pub_;
	tf::TransformListener listener_;

	// Point structure that includes a planar distance calc, time stamp, and number stamp
	struct point_XYZDTO
	{
		float x;
		float y;
		float z;
		float distance;
		ros::Time time_stamp;
		int obstacle; // Defined point state.  0 = Not an obstacle, 1 = Obstacle, 2 = Likely Drivable (unsure), 3 = Too far away and too high/low (unsure)
	};

	//Varibles that will be set via yaml file
	std::string alg_name_;
	std::string map_reg_;
	float robot_height_default_;
	float floor_range_;
	int scan_res_;
	float map_res_;
	float drivable_height_;
	float max_step_height_;
	float min_obj_dist_;
	float max_check_dist_;
	float max_robot_reach_;
	float stale_map_time_;
	float obs_decay_time_;
	std::string obs_decay_type_;
	float obs_decay_factor_;
	float loop_rate_;

	//Robot footprint variables
	geometry_msgs::PolygonStamped current_poly_;
	bool poly_init_;
	bool in_poly_;
	geometry_msgs::PointStamped robot_base_point_;

	//Robot height variables
	geometry_msgs::Point32 current_robot_height_;
	bool robot_height_init_;

	//Map variables
	nav_msgs::OccupancyGrid map_to_publish_;
	sensor_msgs::LaserScan scan_to_publish_;
	std::vector<point_XYZDTO> occupied_list_;
	std::vector<point_XYZDTO> points_checked_;
	bool map_dim_change_;

	int num_of_pts_;
	float max_x_; // The farthest point measured in meters by the lidar in the x direction
	float max_y_; // The farthest point measured in meters by the lidar in the y direction
	float min_x_;
	float min_y_;
	float upper_x_; // The upper bound in meters of the map in the x direction (must be greater than max_x)
	float upper_y_; // The upper bound in meters of the map in the y direction (must be greater than max_y)
	float prev_max_x_;
	float prev_max_y_;
	float prev_min_x_;
	float prev_min_y_;
	float prev_origin_x_;
	float prev_origin_y_;
	float prev_upper_x_;
	float prev_upper_y_;
	int prev_height_;
	int prev_width_;
	int dx_;
	int d_x_;
	int dy_;
	int d_y_;
	std::vector<int> prev_map_data_;

	ros::Time start_time_;
	ros::Time prev_map_build_time_;
	bool map_init_;
	float slope_threshold_;

	// Visualization tool variables
	bool viz_tool_;
	float viz_timer_;
	ros::Time prev_viz_pub_time_;
	pcl::PointCloud<pcl::PointXYZI> viz_cloud_;
	sensor_msgs::PointCloud2 viz_cloud_pub_;


	//Callbacks
	void mainCallback(const sensor_msgs::PointCloud2::ConstPtr& planar_cloud);
	void heightMethod(const sensor_msgs::PointCloud2::ConstPtr&  planar_cloud);
	void slopeMethod(const sensor_msgs::PointCloud2::ConstPtr& planar_cloud);
	void mapBuilder();
	void scanBuilder();
	void mapSizeMaintainer(const pcl::PointXYZI point);
	void mapDiscretizer();
	void convertPoly(const std_msgs::Header new_header);
	void updatePoly(const geometry_msgs::PolygonStamped::ConstPtr& new_poly_stamped);
	void updateHeight(const geometry_msgs::Point32::ConstPtr& new_height);
	void visualizationTool();
	static bool compareDistance(point_XYZDTO p1, point_XYZDTO p2);
};

#endif //LIVE_MAPPER_H
