
#ifndef POINT_IN_POLY_H
#define POINT_IN_POLY_H

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

bool point_in_poly(geometry_msgs::PolygonStamped polygon, geometry_msgs::PointStamped point);

#endif //POINT_IN_POLY_H
