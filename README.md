# nav_3d

This package contains tools for use in 3D navigation.

## Live_mapper

Live_mapper is designed to take a 3D pointcloud2 input and output a 2D obstacle map of the environment for navigation.  It currently uses two methods to analyze the incoming pointcloud for obstacles: Height Method and Slope Method.  

### Height Method

Height method is a simple and efficient method of sifting the pointcloud for points that are at a Z-height that would collide with the robot height profile, or points that a far enough below the ground such that they will be considered a negative obstacle point.  This method is dependent on the assumption that the ground is mostly planar.

### Slope Method

Slope method is a more complex method for iterating through a planar pointcloud to determine obstalce points.  It works on the assumption that the pointcloud received is a planar pointcloud and is close to orthogonal to the robots driving plane.  Its algorithm does slope calculations from point to point to determine obstalce points in the cloud.  Params can be set to tweak the algorithms sensistivity to certain types of obstacles/environments.

## Robot State

Robot state is designed to simply iterate through the robot's TF tree and output a robot footprint (polygon/polygonStamped) for the profile of the robot on the ground as well as the robot height.  Robot height and robot footprint are both used in live_mapper.
