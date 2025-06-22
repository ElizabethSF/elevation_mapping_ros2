# Robot-Centric Elevation Mapping


## runrunxin

### quick start

1. build
```
cd ws_elevation_map_ros2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Note：need to specify the path of kindr and kindr_ros in elevation_mapping/CMakeLists.txt, for example:
```
set(kindr_DIR "/home/snoopy/Autowheelloader/repos/kindr/build/CMakeFiles")
set(kindr_ros_DIR "/home/snoopy/Autowheelloader/repos/kindr_ros/install/kindr_ros/share/kindr_ros/cmake")
```
2. run
```
source install/setup.bash
ros2 launch elevation_mapping_demos my_demo.launch.py
```

### 6月22日
1. change 'elevation_mapping_demos' package from ros1 to ros2 style
2. fixed bugs and make it work on our autowheelloader data.

Key parameters:
For mapping:
1. length_in_x & length_in_x: decide how large the map is
2. ignorePointsUpperThreshold_: ignore the points above
3. ignorePointsInsideMinX_: ignore the points around the robot

For map updates:
| 参数名                                    | 类型     | 默认值      | 含义说明                                                               |
| -------------------------------------- | ------ | -------- | ------------------------------------------------------------------ |
| `min_variance`                         | double | `0.0001` | 地图中每个栅格的最小高度方差，防止估计值过于确定，增强数值稳定性。值越小，表示对当前高度的确信程度越高。               |
| `max_variance`                         | double | `0.01`   | 地图中每个栅格允许的最大高度方差，限制融合传感器数据的不确定性。如果新观测的不确定度高于此值，则不会用于融合。            |
| `mahalanobis_distance_threshold`       | double | `5.0`    | 用于判断新观测与当前地图高度是否一致的马氏距离阈值。低于该值时视为一致并融合，高于则可能被忽略或替换。值越小，系统越敏感，越易更新。 |
| `multi_height_noise`             | double | `0.1` | 多高度观测的噪声估计项，用于融合多个高度观测点。值越小代表新点非常可信，地图对其更新非常谨慎；值越大则允许地图更快适应高度变动（如物体被移除）。 |

调整完这些参数之后（都是增大），物体移开，地图会较快进行更新。


![current_result](data/grid_mapping.png)

## elevation_mapping_ros2
This is a simplified version of the [Robot-Centric Elevation Mapping](https://github.com/ANYbotics/elevation_mapping) package from Anybotics. The main changes are the following:

* Migrated to ROS2 Humble
* Removed deprecated parameter `point_cloud_topic`
* Introduced `input_sources` parameter to specify list of input sources eg. `["top", "front"]`
* Removed implementation of post-processing filters to use directly `grid_map_demos`
* Subscribing to a `nav_msgs/Odometry` topic instead of `geometry_msgs/Pose`
* Removed `robot_base_frame_id` parameter as it is not used
* Removed underlying map handling

Dependencies:
- [grid_map](https://github.com/ANYbotics/grid_map/tree/foxy-devel) - foxy-devel branch
    - filters - `sudo apt-get isntall ros-foxy-filters`
    - nav2_msgs
- [kindr](https://github.com/ANYbotics/kindr)
- [kindr_ros](https://github.com/SivertHavso/kindr_ros/tree/ros2) - ros2 branch
- [Eigen]
To install all dependencies at once, run the provided script:
```
./install_dependencies.sh
```

## Overview

This is a [ROS] package developed for elevation mapping with a mobile robot. The software is designed for (local) navigation tasks with robots which are equipped with a pose estimation (e.g. IMU & odometry) and a distance sensor (e.g. structured light (Kinect, RealSense), laser range sensor, stereo camera). The provided elevation map is limited around the robot and reflects the pose uncertainty that is aggregated through the motion of the robot (robot-centric mapping). This method is developed to explicitly handle drift of the robot pose estimation.

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Péter Fankhauser<br />
Co-Author: Maximilian Wulf<br />
Affiliation: [ANYbotics](https://www.anybotics.com/)<br />
Maintainer: Maximilian Wulf, mwulf@anybotics.com, Magnus Gärtner, mgaertner@anybotics.com<br />**

This projected was initially developed at ETH Zurich (Autonomous Systems Lab & Robotic Systems Lab).

[This work is conducted as part of ANYmal Research, a community to advance legged robotics.](https://www.anymal-research.org/)

<img alt="Elevation Map Example" src="elevation_mapping_demos/doc/elevation_map.jpg" width="700">


Videos of the elevation mapping software in use:

<a alt="StarlETH Kinect elevation mapping" href="https://www.youtube.com/watch?v=I9eP8GrMyNQ"><img src="elevation_mapping_demos/doc/starleth_kinect.jpg" align="left" width="180" ></a>
<a alt="ANYmal outdoor terrain mapping" href="https://www.youtube.com/watch?v=iVMsQPTM65M"><img src="elevation_mapping_demos/doc/anymal_forrest.jpg" align="left" width="180" ></a>
<a alt="ANYmal rough-terrain locomotion planner" href="https://www.youtube.com/watch?v=CpzQu25iLa0"><img src="elevation_mapping_demos/doc/anymal_locomotion_planner.jpg" align="left" width="180" ></a>
<a alt="ANYmal outdoor stair climbing" href="https://www.youtube.com/watch?v=vSveQrJLRTo"><img src="elevation_mapping_demos/doc/anymal_outdoor_stairs.jpg" width="180" ></a>

## Citing

The robot-centric elevation mapping methods used in this software are described in the following paper (available [here](https://doi.org/10.3929/ethz-b-000272110)). If you use this work in an academic context, please cite the following publication(s):

* > P. Fankhauser, M. Bloesch, and M. Hutter,
  > **"Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization"**,
  > in IEEE Robotics and Automation Letters (RA-L), vol. 3, no. 4, pp. 3019–3026, 2018. ([PDF](http://dx.doi.org/10.1109/LRA.2018.2849506))

        @article{Fankhauser2018ProbabilisticTerrainMapping,
          author = {Fankhauser, P{\'{e}}ter and Bloesch, Michael and Hutter, Marco},
          doi = {10.1109/LRA.2018.2849506},
          title = {Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization},
          journal = {IEEE Robotics and Automation Letters (RA-L)},
          volume = {3},
          number = {4},
          pages = {3019--3026},
          year = {2018}
        }

* > P. Fankhauser, M. Bloesch, C. Gehring, M. Hutter, and R. Siegwart,
  > **"Robot-Centric Elevation Mapping with Uncertainty Estimates"**,
  > in International Conference on Climbing and Walking Robots (CLAWAR), 2014. ([PDF](http://dx.doi.org/10.3929/ethz-a-010173654))

        @inproceedings{Fankhauser2014RobotCentricElevationMapping,
          author = {Fankhauser, P\'{e}ter and Bloesch, Michael and Gehring, Christian and Hutter, Marco and Siegwart, Roland},
          title = {Robot-Centric Elevation Mapping with Uncertainty Estimates},
          booktitle = {International Conference on Climbing and Walking Robots (CLAWAR)},
          year = {2014}
        }

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionally, the Robot-Centric Elevation Mapping depends on following software:

- [Grid Map](https://github.com/anybotics/grid_map) (grid map library for mobile robots)
- [kindr](http://github.com/anybotics/kindr) (kinematics and dynamics library for robotics),
- [kindr_ros](https://github.com/anybotics/kindr_ros) (ROS wrapper for kindr),
- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing),
- [Eigen](http://eigen.tuxfamily.org) (linear algebra library).


### Building

In order to install the Robot-Centric Elevation Mapping, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd ~/ros2_ws/src
    git clone https://github.com/anybotics/elevation_mapping.git
    cd ..
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


### Unit Tests

**NOTE: This was not implemented in this fork.**

Build tests with
    
    roscd elevation_mapping
    catkin build --catkin-make-args run_tests -- --this
   
Run the tests with
    
     rostest elevation_mapping elevation_mapping.test -t
    

## Basic Usage

In order to get the Robot-Centric Elevation Mapping to run with your robot, you will need to adapt a few parameters. It is the easiest if duplicate and adapt all the parameter files that you need to change from the `elevation_mapping_demos` package (e.g. the `simple_demo` example). These are specifically the parameter files in `config` and the launch file from the `launch` folder.

### TurtleBot3 Waffle Simulation

**NOTE: This was not implemented in this fork.**

A running example is provided, making use of the Turtlebot3 simulation environment. This example can be used to test elevation mapping, as a starting point for further integration.

To start with, the Turtlebot3 simulation dependencies need to be installed:

    sudo apt install ros-melodic-turtlebot3*

The elevation mapping demo together with the turtlebot3 simulation can be started with

    roslaunch elevation_mapping_demos turtlesim3_waffle_demo.launch

To control the robot with a keyboard, a new terminal window needs to be opened (remember to source your ROS environment). Then run

    export TURTLEBOT3_MODEL=waffle
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Velocity inputs can be sent to the robot by pressing the keys `a`, `w`,`d`, `x`. To stop the robot completely, press `s`.

### Simple Demo & Ground Truth Demo

**NOTE: This was not implemented in this fork.**

A .ply is published as static pointcloud, elevation_mapping subscribes to it and publishes the elevation map. You can visualize it through rviz. 
For visualization, select `/elevation_mapping/elevation_map_raw`. 

**Note**. You might need to toggle the visibility of the grid_map_plugin to visualize it. 
```bash
roslaunch elevation_mapping_demos ground_truth_demo.launch
```

While ground truth demo estimates the height in map frame, simple demo sets up a more realistic deployment scenario. Here, the elevation_map is configured to track a base frame.
To get started, we suggest to play around and also visualize other published topics, such as `/elevation_mapping/elevation_map_raw` and change the height layer to another layer, e.g `elevation_inpainted`.

## Nodes

### Node: elevation_mapping

This is the main Robot-Centric Elevation Mapping node. It uses the distance sensor measurements and the pose and covariance of the robot to generate an elevation map with variance estimates.

#### Subscribed Topics

* **`/points`** ([sensor_msgs/PointCloud2])

    The distance measurements.

* **`/pose`** ([geometry_msgs/PoseWithCovarianceStamped])

    The robot pose and covariance.

* **`/tf`** ([tf2_msgs/TFMessage])

    The transformation tree.


#### Published Topics

* **`elevation_map`** ([grid_map_msgs/GridMap])

    The elevation map, layers include mainly elevation and variance.

* **`visibility_cleanup_map`** ([grid_map_msgs/GridMap])

    The cleaned-up elevation map obtained from ray-tracing, published only if `enable_visibility_cleanup` if set to `True`.


#### Services

* **`clear_map`** ([std_srvs/Empty])

    Initiates clearing of the entire map for resetting purposes. Trigger the map clearing with

        rosservice call /elevation_mapping/clear_map

* **`masked_replace`** ([grid_map_msgs/SetGridMap])

    Allows for setting the individual layers of the elevation map through a service call. The layer mask can be used to only set certain cells and not the entire map. Cells containing NAN in the mask are not set, all the others are set. If the layer mask is not supplied, the entire map will be set in the intersection of both maps. The provided map can be of different size and position than the map that will be altered. An example service call to set some cells marked with a mask in the elevation layer to 0.5 is

        rosservice call /elevation_mapping/masked_replace "map:
          info:
            header:
              seq: 3
              stamp: {secs: 3, nsecs: 80000000}
              frame_id: 'odom'
            resolution: 0.1
            length_x: 0.3
            length_y: 0.3
            pose:
              position: {x: 5.0, y: 0.0, z: 0.0}
              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          layers: [elevation,mask]
          basic_layers: [elevation]
          data:
          - layout:
              dim:
              - {label: 'column_index', size: 3, stride: 9}
              - {label: 'row_index', size: 3, stride: 3}
              data_offset: 0
            data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
          - layout:
              dim:
              - {label: 'column_index', size: 3, stride: 9}
              - {label: 'row_index', size: 3, stride: 3}
              data_offset: 0
            data: [0, 0, 0, .NAN, .NAN, .NAN, 0, 0, 0]
          outer_start_index: 0
          inner_start_index: 0"

* **`save_map`** ([grid_map_msgs/ProcessFile])

    Saves the current fused grid map and raw grid map to rosbag files. Field `topic_name` must be a base name, i.e. no leading slash character (/). If field `topic_name` is empty, then `elevation_map` is used per default. Example with default topic name

        rosservice call /elevation_mapping/save_map "file_path: '/home/integration/elevation_map.bag' topic_name: ''"

* **`load_map`** ([grid_map_msgs/ProcessFile])

    Loads the fused grid map and raw grid map from rosbag files. Field `topic_name` must be a base name, i.e. no leading slash character (/). If field `topic_name` is empty, then `elevation_map` is used per default. Example with default topic name

        rosservice call /elevation_mapping/load_map "file_path: '/home/integration/elevation_map.bag' topic_name: ''"

* **`disable_updates`** ([std_srvs/Empty])

    Stops updating the elevation map with sensor input. Trigger the update stopping with

        rosservice call /elevation_mapping/disable_updates {}

* **`enable_updates`** ([std_srvs/Empty])

    Start updating the elevation map with sensor input. Trigger the update starting with

        rosservice call /elevation_mapping/enable_updates {}

#### Parameters
    
* **`input_sources`** (list of input sources, default: none)

    Here you specify your inputs to elevation mapping, currently "pointcloud" inputs are supported. 
    
    Example configuration:
    ```yaml
    input_sources:
        front: # A name to identify the input source
          type: pointcloud # Supported types: pointcloud
          topic: /lidar_front/depth/points
          queue_size: 1
          publish_on_update: true # Wheter to publish the elevation map after a callback from this source. 
        rear:
          type: pointcloud
          topic: /lidar_rear/depth/points
          queue_size: 5
          publish_on_update: false
    ```
    No input sources can be configured with an empty array:
    ```yaml
    input_sources: []
    ```
* **`robot_odom_topic`** (string, default: "/odom")

    The name of the robot odometry topic.

* **`robot_base_frame_id`** (string, default: "base_link")

    The id of the robot base tf frame.

* **`map_frame_id`** (string, default: "map")

    The id of the tf frame of the elevation map.

* **`track_point_frame_id`** (string, default: "base_link")

    The elevation map is moved along with the robot following a *track point*. This is the id of the tf frame in which the track point is defined.

* **`track_point_x`**, **`track_point_y`**, **`track_point_z`** (double, default: 0.0, 0.0, 0.0)

    The elevation map is moved along with the robot following a *track point*. This is the position of the track point in the `track_point_frame_id`.

* **`robot_odom_cache_size`** (int, default: 200, min: 0)

    The size of the robot odom cache.

* **`min_update_rate`** (double, default: 2.0)

    The mininum update rate (in Hz) at which the elevation map is updated either from new measurements or the robot pose estimates.

* **`length_in_x`**, **`length_in_y`** (double, default: 1.5, min: 0.0)

    The size (in m) of the elevation map.

* **`position_x`**, **`position_y`** (double, default: 0.0)

    The position of the elevation map center, in the elevation map frame. This parameter sets the planar position offsets between the generated elevation map and the frame in which it is published (`map_frame_id`). It is only useful if no `track_point_frame_id` parameter is used.

* **`resolution`** (double, default: 0.01, min: 0.0)

    The resolution (cell size in m/cell) of the elevation map.

* **`min_variance`**, **`max_variance`** (double, default: 9.0e-6, 0.01)

    The minimum and maximum values for the elevation map variance data.

* **`mahalanobis_distance_threshold`** (double, default: 2.5)

    Each cell in the elevation map has an uncertainty for its height value. Depending on the Mahalonobis distance of the existing height distribution and the new measurements, the incoming data is fused with the existing estimate, overwritten, or ignored. This parameter determines the threshold on the Mahalanobis distance which determines how the incoming measurements are processed.

* **`sensor_processor/ignore_points_above`** (double, default: inf)
    A hard threshold on the height of points introduced by the depth sensor. Points with a height over this threshold will not be considered valid during the data collection step.

* **`sensor_processor/ignore_points_below`** (double, default: -inf)
    A hard threshold on the height of points introduced by the depth sensor. Points with a height below this threshold will not be considered valid during the data collection step.

* **`multi_height_noise`** (double, default: 9.0e-7)

    Noise added to measurements that are higher than the current elevation map at that particular position. This noise-adding process is only performed if a point falls over the Mahalanobis distance threshold. A higher value is useful to adapt faster to dynamic environments (e.g., moving objects), but might cause more noise in the height estimation.

* **`min_horizontal_variance`**, **`max_horizontal_variance`** (double, default: pow(resolution / 2.0, 2), 0.5)

    The minimum and maximum values for the elevation map horizontal variance data.

* **`enable_visibility_cleanup`** (bool, default: true)

    Enable/disable a separate thread that removes elements from the map which are not visible anymore, by means of ray-tracing, originating from the sensor frame.

* **`visibility_cleanup_rate`** (double, default: 1.0)

    The rate (in Hz) at which the visibility clean-up is performed.

* **`enable_continuous_cleanup`** (bool, default: false)

    Enable/disable a continuous clean-up of the elevation map. If enabled, on arrival of each new sensor data the elevation map will be cleared and filled up only with the latest data from the sensor. When continuous clean-up is enabled, visibility clean-up will automatically be disabled since it is not needed in this case.

* **`scanning_duration`** (double, default: 1.0)

    The sensor's scanning duration (in s) which is used for the visibility cleanup. Set this roughly to the duration it takes between two consecutive full scans (e.g. 0.033 for a ToF camera with 30 Hz, or 3 s for a rotating laser scanner). Depending on how dense or sparse your scans are, increase or reduce the scanning duration. Smaller values lead to faster dynamic object removal and bigger values help to reduce faulty map cleanups.


## Changelog

See [Changelog]

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/anybotics/elevation_mapping/issues).

[Changelog]: CHANGELOG.rst
[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[grid_map_msgs/GridMap]: https://github.com/anybotics/grid_map/blob/master/grid_map_msgs/msg/GridMap.msg
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[geometry_msgs/PoseWithCovarianceStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[tf2_msgs/TFMessage]: http://docs.ros.org/en/noetic/api/tf2_msgs/html/msg/TFMessage.html
[std_srvs/Empty]: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
[grid_map_msgs/GetGridMap]: https://github.com/anybotics/grid_map/blob/master/grid_map_msgs/srv/GetGridMap.srv
[grid_map_msgs/ProcessFile]: https://github.com/ANYbotics/grid_map/blob/master/grid_map_msgs/srv/ProcessFile.srv
