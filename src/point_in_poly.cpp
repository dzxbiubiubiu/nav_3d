
///////////////////////////////////////////////////////////////////////////////
//      Title     : point_in_poly.cpp
//      Project   : nav_3d
//      Created   : 1/23/2018
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

#include "point_in_poly.h"

bool point_in_poly(geometry_msgs::PolygonStamped polygon, geometry_msgs::PointStamped point) {
	/* This is a method of determining if a given point is within a given polygon. 
	It assumes that the given point and polygon are both stamped instances.

	NOTE: If the point and polygon are in different frames then it wants to transform the point into the same frame.
	However, if this is being called very often the function transformPoint will slow performance greatly. 
	In a case like that, it is better to ensure the point and polgyon are in the same frame already. */

	int n = polygon.polygon.points.size();

	bool result = false;

	float p1x = polygon.polygon.points[0].x;
	float p1y = polygon.polygon.points[0].y;
	float p2x;
	float p2y;
	float xinters;

	for(int i=0; i<polygon.polygon.points.size(); i++) {
		p2x = polygon.polygon.points[i].x;
		p2y = polygon.polygon.points[i].y;
		
		if(point.point.y > std::min(p1y, p2y)) {
			if(point.point.y <= std::max(p1y, p2y)) {
				if(point.point.x <= std::max(p1x, p2x)) {
					
					if(p1y != p2y)
						xinters = (point.point.y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
					
					if((p1x == p2x) || (point.point.x <= xinters))
						result = !result;
				}
			}
		}
		p1x = p2x;
		p1y = p2y;
	}
	return(result);
}
