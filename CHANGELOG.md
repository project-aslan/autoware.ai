```

└───autoware.ai: Fork from autoware-ai open source software for self-driving vehicles
|     └───localization
|     │  └───lib: 
|     |  |    ndt_cpu: Developed by Point Cloud Library (PCL) and Autoware 1.10.0. (https://github.com/PointCloudLibrary/pcl). 
|     |  |    Changes made:
|     |  |          * fixed copyrights to include Point Cloud Library (PCL)
|     |  |
|     |  |    pcl_omp_registration: Originally developed by Point Cloud Library (PCL) (https://github.com/PointCloudLibrary/pcl). 
|     |  |     This package has remained unchanged.
|     │  | 
|     │  └───packages/lidar_localizer: 
|     │       The node save_pcd is a Project-Aslan specific developement.
|     │       The nodes ndt_mapping, ndt_matching, ndt_matching_monitor were originally included in Autoware 1.10.0. 
|     │       Changes made:
|     │  		* Launch pose_relay and vel_relay from topic_tools/relay (https://github.com/ros/ros_comm/tree/kinetic-devel/tools) 
|     │  		  alongside ndt_matching (http://wiki.ros.org/topic_tools/relay) 
|     │  		* ndt_mapping: Doesn't support GPU for NDT
|     │  		* ndt_matching:  Doesn't support GPU for NDT. Publishing predict_pose_error 
|     |			(the difference between ndt_pose and predict_pose)
|     │ 
|     └───mapping
|     |   └───map_tf_generator: This package has remained unchanged.
|     |   |     
|     |   └───pcd_loader: Originally included in Autoware.ai 1.10.0 under package /map_file. Changes made:
|     |		* Doesn't support vector maps. Project Aslan is tested and proven stable with point cloud maps only.
|     | 
|     └───planning
|     |   └───mission/packages/lane_planner
|     │   |	The node traffic_waypoints is Project-Aslan specific development.
|     |   |	[Feature:] This node is publishing the traffic waypoints array.
|     │   |	The node lane_select is included in Autoware.ai 1.10.0.
|     |   |     
|     |   └───motion
|     |       └───astar_planner: Included in Autoware.ai 1.10.0. Changes made:
|     │       │   Changes made:
|     │       │     * velocity_set parameters: Added radar detection specific launch parameters, added current_velocity 
|     │       │     input topic definition, added obstacle detection additional launch parameters, specified
|     │       │     default parameters setting after on-vehicle testing, removed crosswalk detection.
|     │       │     * velocity_set: 
|     |       |         * [Feature] Configurable speed input topic
|     │       │         * [Feature] Radar obstacle detection
|     │       │         * [Feature] Emergency reaction to obstacle detection from radar 
|     │       │         * Calibration for deceleration and search distance after on-vehicle testing 
|     │       │         * [Feature] Emergency stop, obstacle detection, obstacle removed
|     │       │         * Doesn't support vector maps and crosswalk detection
|     │       │     
|     |       └───waypoint_follower: Included in Autoware.ai 1.10.0. Changes made:
|     │       │     * [Feature] pure_pursuit - Vehicle speed conditioning to fit the requirements of 
|     │       │     Project-Aslan: 9mps speed cap.
|     │       │     * [Feature] twist_filter - Obstacle removed callback reaction, added zero_twist counter after 
|     │       │     on-vehicle testing to capture unexpected message dropping to 0
|     │       │     * twist_gate: Doesn't support mode and gear command
|     │       │     * Fixed warning messages when building
|     │       │     
|     |       └───waypoint_maker: Originally included in Autoware.ai 1.10.0. Changes made:
|     │             * waypoint_marker_publisher: Doesn't support traffic lights
|     │             * Fixed warning messages when building
|     │             * Parameters calibration from on-vehicle testing
|     |
|     └───filters
|          └───ray_ground_filter: Originally included in Autoware.ai 1.10.0 under points_preprosessor. 
|          |    Changes made:
|          |       * Parameters calibration from on-vehicle testing
|          |  
|          └───voxel_grid_filter: Originally included in Autoware.ai 1.10.0 under points_downsampler. 
|       	Changes made:
|       	   * Parameters calibration from on-vehicle testing
|

```

