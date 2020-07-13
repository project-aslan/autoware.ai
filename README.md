### autoware.ai:
Open source software for self-driving vehicles.  

#### localization/lidar_localizer:
For the vehicle to accurately estimate its position within an environment, autoware.ai is suggesting a point cloud map-based localisation method which StreetDrone has evaluated in terms of accuracy, resilience to error and noise, coverage and cost, before integrating in Aslan. This localization approach is using the point cloud matching method, Normal Distribution Transform or NDT in which a LiDAR scan from the vehicle is being compared to the LiDAR scan of the map, in order to calculate its position in that map.   
**Added Features for Aslan:**   
* The node save_pcd is a Project-Aslan specific development. This package is responsible for saving a point cloud map at a specific path. It's being automatically run inside ndt_mapping.
* Launch pose_relay and vel_relay from [topic_tools/relay](https://github.com/ros/ros_comm/tree/kinetic-devel/tools) alongside ndt_matching [(http://wiki.ros.org/topic_tools/relay)](http://wiki.ros.org/topic_tools/relay)
* ndt_matching: Publish predict_pose_error (the difference between ndt_pose and predict_pose)   
* More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/localization/packages/lidar_localizer)

#### mission/lane_planner: 
This package is responsible for extracting the route for the vehicle to follow. It is using pre-defined waypoints in the form of lane arrays and then it is drawing the route by finding the closest waypoint from the vehicle's current position in the map.   
**Added Features for Aslan:**  
* The node traffic_waypoints is Project Aslan specific development. This node is responsible for publishing the traffic waypoints array from the lane waypoints array.   
* More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/planning/mission/packages/lane_planner)

#### motion/astar_planner:
This package is based on the A* search algorithm for path planning. The A* algorithm is using a cost function for finding the optimal way of reaching the next target waypoint. It's considered to be the best planning algorithm for finding the shortest path to target.  
**Added Features for Aslan:**  
* Radar obstacle detection
* Emergency reaction and emergency stop from radar obstacle detection
* Deceleration and search distance calibration
* Added obstacle removed reaction
* Parameters configuration: Added radar detection specific launch parameters, added current_velocity input topic definition, added obstacle detection additional launch parameters, specified default parameters setting after on-vehicle testing, removed crosswalk detection.   
* Configurable speed input topic
* More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/planning/motion/packages/astar_planner)

#### motion/waypoint_follower: 
This package is responsible for calculating the final velocity command for the vehicle, by using the output from the path planning module and fitting a curve between the car and the next target waypoint, based on the lookahead distance specified.  
**Added Features for Aslan:**  
* Added emergency stop reaction
* Vehicle speed conditioning to fit the requirements of Project Aslan: 9mps speed cap (at pure_pursuit).  
* Obstacle removed callback reaction, added zero_twist counter after on-vehicle testing to capture unexpected message dropping to 0 (at twist filter). 
* More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/planning/motion/packages/waypoint_follower)

#### motion/waypoint_maker: 
This package is responsible for extracting and then loading the waypoints for the vehicle to follow. Each waypoint is associated with (x,y,z) coordinates relative to a common reference point and a velocity value.  
* More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/planning/motion/packages/waypoint_maker)

#### filters
* voxel_grid filter: This package is responsible for downsampling the input point cloud data produced by the lidar using the voxel grid filtering technique. More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/filters/voxel_grid_filter)
* ray_ground_filter: This package is responsible for filtering the output point cloud from a sensor (either lidar or radar). It is responsible for performing ground removal, by masking the point cloud received. More information [here](https://github.com/project-aslan/autoware.ai/tree/aslan-dev/filters/ray_ground_filter)


#### Overall changes made:
* All path planning and controls parameters are configured to default values after on-vehicle testing
* NDT doesn't include GPU support
* Aslan doesn't include vector maps support. Project Aslan is tested and proven stable with point cloud maps only.   
* Aslan doesn't support crosswalk detection, traffic lights, mode and gear command
