/*
 * ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <cmath>
#include <string>

#include <grid_map_msgs/msg/grid_map.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/point_stamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapping.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping
{

  ElevationMapping::ElevationMapping(std::shared_ptr<rclcpp::Node> &nodeHandle) : nodeHandle_(nodeHandle),
                                                                                  inputSources_(nodeHandle_),
                                                                                  robotOdomCacheSize_(200),
                                                                                  map_(nodeHandle),
                                                                                  robotMotionMapUpdater_(nodeHandle),
                                                                                  ignoreRobotMotionUpdates_(false),
                                                                                  updatesEnabled_(true),
                                                                                  maxNoUpdateDuration_(rclcpp::Duration::from_seconds(0.0)),
                                                                                  timeTolerance_(rclcpp::Duration::from_seconds(0.0)),
                                                                                  visibilityCleanupTimerDuration_(rclcpp::Duration::from_seconds(0.0)),
                                                                                  receivedFirstMatchingPointcloudAndPose_(false),
                                                                                  lengthInXInitSubmap_(1.2),
                                                                                  lengthInYInitSubmap_(1.8),
                                                                                  marginInitSubmap_(0.3),
                                                                                  initSubmapHeightOffset_(0.0)
  {
#ifndef NDEBUG
    // Print a warning if built in debug.
    RCLCPP_WARN(nodeHandle_->get_logger(), "CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif

    RCLCPP_INFO(nodeHandle_->get_logger(), "Elevation mapping node started.");

    readParameters();
    setupSubscribers();
    setupServices();
    setupTimers();

    transformBuffer_ = std::make_shared<tf2_ros::Buffer>(nodeHandle_->get_clock());
    transformListener_ = std::make_shared<tf2_ros::TransformListener>(*transformBuffer_);

    initialize();

    RCLCPP_INFO(nodeHandle_->get_logger(), "Successfully launched node.");
  }

  void ElevationMapping::setupSubscribers()
  {
    auto res = nodeHandle_->get_topic_names_and_types();
    for (auto a : res)
    {
      RCLCPP_INFO(nodeHandle_->get_logger(), "topic: %s", a.first.c_str());
    }

    const bool configuredInputSources = inputSources_.configureFromRos("input_sources");
    if (configuredInputSources)
    {
      inputSources_.registerCallbacks(*this, std::make_pair("pointcloud", &ElevationMapping::pointCloudCallback));
    }
    else
    {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Input sources not configured!");
    }

    if (!robotOdomTopic_.empty())
    {
      robotOdomSubscriber_.subscribe(nodeHandle_, robotOdomTopic_);
      robotOdomCache_.connectInput(robotOdomSubscriber_);
      robotOdomCache_.setCacheSize(robotOdomCacheSize_);
    }
    else
    {
      ignoreRobotMotionUpdates_ = true;
    }
  }

  void ElevationMapping::setupServices()
  {
    clearMapService_ = nodeHandle_->create_service<std_srvs::srv::Empty>("clear_map", std::bind(&ElevationMapping::clearMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    enableUpdatesService_ = nodeHandle_->create_service<std_srvs::srv::Empty>("enable_updates", std::bind(&ElevationMapping::enableUpdatesServiceCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    disableUpdatesService_ = nodeHandle_->create_service<std_srvs::srv::Empty>("disable_updates", std::bind(&ElevationMapping::disableUpdatesServiceCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    maskedReplaceService_ = nodeHandle_->create_service<grid_map_msgs::srv::SetGridMap>("masked_replace", std::bind(&ElevationMapping::maskedReplaceServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    saveMapService_ = nodeHandle_->create_service<grid_map_msgs::srv::ProcessFile>("save_map", std::bind(&ElevationMapping::saveMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    loadMapService_ = nodeHandle_->create_service<grid_map_msgs::srv::ProcessFile>("load_map", std::bind(&ElevationMapping::loadMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void ElevationMapping::setupTimers()
  {
    mapUpdateTimer_ = rclcpp::create_timer(
        nodeHandle_, nodeHandle_->get_clock(), maxNoUpdateDuration_,
        std::bind(&ElevationMapping::mapUpdateTimerCallback, this));
    mapUpdateTimer_->cancel();

    // Create visibility cleanup timer. Visibility clean-up does not help when continuous clean-up is enabled.
    if (map_.enableVisibilityCleanup_ && (visibilityCleanupTimerDuration_.seconds() != 0.0) && !map_.enableContinuousCleanup_)
    {
      visibilityCleanupTimer_ = rclcpp::create_timer(
          nodeHandle_, nodeHandle_->get_clock(), visibilityCleanupTimerDuration_,
          std::bind(&ElevationMapping::visibilityCleanupCallback, this),
          visibilityCleanupGroup_);
      visibilityCleanupTimer_->cancel();
    }
  }

  ElevationMapping::~ElevationMapping()
  {
    visibilityCleanupTimer_->cancel();
    rclcpp::shutdown();
  }

  bool ElevationMapping::readParameters()
  {
    // ElevationMapping parameters.
    // FIXME: Fix for case when robot pose is not defined
    robotOdomTopic_ = nodeHandle_->declare_parameter("robot_odom_topic", std::string("/odom"));
    nodeHandle_->declare_parameter("robot_base_frame_id", std::string("base_link"));
    trackPointFrameId_ = nodeHandle_->declare_parameter("track_point_frame_id", std::string("base_link"));
    trackPoint_.x() = nodeHandle_->declare_parameter("track_point_x", 0.0);
    trackPoint_.y() = nodeHandle_->declare_parameter("track_point_y", 0.0);
    trackPoint_.z() = nodeHandle_->declare_parameter("track_point_z", 0.0);
    robotOdomCacheSize_ = nodeHandle_->declare_parameter("robot_odom_cache_size", 200);

    assert(robotOdomCacheSize_ > 0);

    double minUpdateRate = nodeHandle_->declare_parameter("min_update_rate", 2.0);
    if (minUpdateRate == 0.0)
    {
      maxNoUpdateDuration_ = rclcpp::Duration::from_seconds(0);
      RCLCPP_WARN(nodeHandle_->get_logger(), "Rate for publishing the map is zero.");
    }
    else
    {
      maxNoUpdateDuration_ = rclcpp::Duration::from_seconds(1.0 / minUpdateRate);
    }
    assert(maxNoUpdateDuration_.seconds() != 0.0);

    double timeTolerance = nodeHandle_->declare_parameter("time_tolerance", 0.0);
    timeTolerance_ = rclcpp::Duration::from_seconds(timeTolerance);

    double visibilityCleanupRate = nodeHandle_->declare_parameter("visibility_cleanup_rate", 1.0);
    if (visibilityCleanupRate == 0.0)
    {
      visibilityCleanupTimerDuration_ = rclcpp::Duration::from_seconds(0.0);
      RCLCPP_WARN(nodeHandle_->get_logger(), "Rate for visibility cleanup is zero and therefore disabled.");
    }
    else
    {
      visibilityCleanupTimerDuration_ = rclcpp::Duration::from_seconds(1.0 / visibilityCleanupRate);
      map_.visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
    }

    // Robot motion map updater parameters.
    if (!robotMotionMapUpdater_.readParameters())
    {
      return false;
    }

    return true;
  }

  bool ElevationMapping::initialize()
  {
    RCLCPP_INFO(nodeHandle_->get_logger(), "Elevation mapping node initializing ... ");

    visibilityCleanupGroup_ = nodeHandle_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::sleep_for(std::chrono::seconds(1)); // Need this to get the TF caches fill up.
    // mapUpdateTimer_->reset();
    // visibilityCleanupTimer_->reset();
    initializeElevationMap();
    return true;
  }

  void ElevationMapping::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointCloudMsg, bool publishPointCloud,
                                            const SensorProcessorBase::Ptr &sensorProcessor_)
  {
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Processing data from: %s", pointCloudMsg->header.frame_id.c_str());
    if (!updatesEnabled_)
    {
      auto clock = nodeHandle_->get_clock();
      RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(), *(clock), 10, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
      if (publishPointCloud)
      {
        map_.setTimestamp(nodeHandle_->get_clock()->now());
        map_.publishRawElevationMap();
      }
      return;
    }

    // Check if point cloud has corresponding robot pose at the beginning
    if (!receivedFirstMatchingPointcloudAndPose_)
    {
      const double oldestPoseTime = robotOdomCache_.getOldestTime().seconds();
      const double currentPointCloudTime = rclcpp::Time(pointCloudMsg->header.stamp).seconds();

      if (currentPointCloudTime < oldestPoseTime)
      {
        auto clock = nodeHandle_->get_clock();
        RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(), *(clock), 5, "No corresponding point cloud and pose are found. Waiting for first match. (Warning message is throttled, 5s.)");
        return;
      }
      else
      {
        RCLCPP_INFO(nodeHandle_->get_logger(), "First corresponding point cloud and pose found, elevation mapping started. ");
        receivedFirstMatchingPointcloudAndPose_ = true;
      }
    }

    mapUpdateTimer_->cancel();

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud.
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);

    PointCloudType::Ptr pointCloud(new PointCloudType);
    pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
    lastPointCloudUpdateTime_ = rclcpp::Time(1000 * pointCloud->header.stamp, RCL_ROS_TIME);

    RCLCPP_DEBUG(nodeHandle_->get_logger(), "ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

    // Get robot pose covariance matrix at timestamp of point cloud.
    Eigen::Matrix<double, 6, 6> robotPoseCovariance;
    robotPoseCovariance.setZero();
    if (!ignoreRobotMotionUpdates_)
    {
      std::shared_ptr<const nav_msgs::msg::Odometry> odomMessage = robotOdomCache_.getElemBeforeTime(lastPointCloudUpdateTime_);
      if (!odomMessage)
      {
        // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
        if (robotOdomCache_.getOldestTime().seconds() > lastPointCloudUpdateTime_.seconds())
        {
          RCLCPP_ERROR(nodeHandle_->get_logger(), "The oldest pose available is at %f, requested pose at %f", robotOdomCache_.getOldestTime().seconds(),
                       lastPointCloudUpdateTime_.seconds());
        }
        else
        {
          RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.seconds());
        }
        return;
      }
      const geometry_msgs::msg::PoseWithCovariance poseMessage = odomMessage->pose;
      robotPoseCovariance = Eigen::Map<const Eigen::MatrixXd>(poseMessage.covariance.data(), 6, 6);
    }

    // Process point cloud.
    PointCloudType::Ptr pointCloudProcessed(new PointCloudType);
    Eigen::VectorXf measurementVariances;
    if (!sensorProcessor_->process(pointCloud, robotPoseCovariance, pointCloudProcessed, measurementVariances,
                                   pointCloudMsg->header.frame_id))
    {
      if (!sensorProcessor_->isTfAvailableInBuffer())
      {
        rclcpp::Clock clock;
        RCLCPP_INFO_THROTTLE(nodeHandle_->get_logger(), clock, 10, "Waiting for tf transformation to be available. (Message is throttled, 10s.)");
        return;
      }
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Point cloud could not be processed."); // TODO: what causes this issue
      mapUpdateTimer_->reset();
      return;
    }

    std::unique_lock<std::recursive_mutex> scopedLock(map_.getRawDataMutex());

    // Update map location.
    updateMapLocation();

    // Update map from motion prediction.
    if (!updatePrediction(lastPointCloudUpdateTime_))
    {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Updating process noise failed.");
      mapUpdateTimer_->reset();
      return;
    }

    // Clear the map if continuous clean-up was enabled.
    if (map_.enableContinuousCleanup_)
    {
      RCLCPP_DEBUG(nodeHandle_->get_logger(), "Clearing elevation map before adding new point cloud.");
      map_.clear();
    }

    // Add point cloud to elevation map.
    if (!map_.add(pointCloudProcessed, measurementVariances, lastPointCloudUpdateTime_,
                  Eigen::Affine3d(sensorProcessor_->transformationSensorToMap_)))
    {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Adding point cloud to elevation map failed.");
      mapUpdateTimer_->reset();
      return;
    }

    if (publishPointCloud)
    {
      RCLCPP_DEBUG(nodeHandle_->get_logger(), "Publishing pcl.");
      // Publish elevation map.
      map_.publishRawElevationMap();
    }

    mapUpdateTimer_->reset();
  }

  void ElevationMapping::mapUpdateTimerCallback()
  {

    if (!updatesEnabled_)
    {
      rclcpp::Clock clock;
      RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(), clock, 10, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
      map_.setTimestamp(nodeHandle_->get_clock()->now());
      map_.publishRawElevationMap();
      return;
    }

    rclcpp::Time time = rclcpp::Clock(RCL_ROS_TIME).now();
    if ((lastPointCloudUpdateTime_ - time) <= maxNoUpdateDuration_)
    { // there were updates from sensordata, no need to force an update.
      return;
    }
    rclcpp::Clock clock;
    RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(), clock, 5, "Elevation map is updated without data from the sensor. (Warning message is throttled, 5s.)");

    std::unique_lock<std::recursive_mutex> scopedLock(map_.getRawDataMutex());

    mapUpdateTimer_->cancel();

    // Update map from motion prediction.
    if (!updatePrediction(time))
    {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Updating process noise failed.");
      mapUpdateTimer_->reset();
      return;
    }

    // Publish elevation map.
    map_.publishRawElevationMap();
    mapUpdateTimer_->reset();
  }

  void ElevationMapping::visibilityCleanupCallback()
  {

    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map is running visibility cleanup.");
    // Copy constructors for thread-safety.x
    map_.visibilityCleanup(rclcpp::Time(lastPointCloudUpdateTime_));
  }

  bool ElevationMapping::updatePrediction(const rclcpp::Time &time)
  {
    if (ignoreRobotMotionUpdates_)
    {
      return true;
    }

    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Updating map with latest prediction from time %f.", robotOdomCache_.getLatestTime().seconds());

    if (time + timeTolerance_ < map_.getTimeOfLastUpdate())
    {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Requested update with time stamp %f, but time of last update was %f.", time.seconds(), map_.getTimeOfLastUpdate().seconds());
      return false;
    }
    else if (time < map_.getTimeOfLastUpdate())
    {
      RCLCPP_DEBUG(nodeHandle_->get_logger(), "Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.seconds(),
                   map_.getTimeOfLastUpdate().seconds());
      return true;
    }

    // Get robot pose at requested time.
    std::shared_ptr<const nav_msgs::msg::Odometry> odomMessage = robotOdomCache_.getElemBeforeTime(time);
    if (!odomMessage)
    {
      // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
      if (robotOdomCache_.getOldestTime().seconds() > lastPointCloudUpdateTime_.seconds())
      {
        RCLCPP_ERROR(nodeHandle_->get_logger(), "The oldest pose available is at %f, requested pose at %f", robotOdomCache_.getOldestTime().seconds(),
                     lastPointCloudUpdateTime_.seconds());
      }
      else
      {
        RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.seconds());
      }
      return false;
    }

    const geometry_msgs::msg::PoseWithCovariance poseMessage = odomMessage->pose;
    kindr::HomTransformQuatD robotPose;
    kindr_ros::convertFromRosGeometryMsg(poseMessage.pose, robotPose);
    // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
    Eigen::Matrix<double, 6, 6> robotPoseCovariance =
        Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage.covariance.data(), 6, 6);

    // Compute map variance update from motion prediction.
    robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);

    return true;
  }

  bool ElevationMapping::updateMapLocation()
  {
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map is checked for relocalization.");

    geometry_msgs::msg::PointStamped trackPoint;
    trackPoint.header.frame_id = trackPointFrameId_;
    trackPoint.header.stamp = rclcpp::Time(0);
    kindr_ros::convertToRosGeometryMsg(trackPoint_, trackPoint.point);
    geometry_msgs::msg::PointStamped trackPointTransformed;

    try
    {
      trackPointTransformed = transformBuffer_->transform(trackPoint, map_.getFrameId());
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "%s", ex.what());
      return false;
    }

    kindr::Position3D position3d;
    kindr_ros::convertFromRosGeometryMsg(trackPointTransformed.point, position3d);
    grid_map::Position position = position3d.vector().head(2);
    map_.move(position);
    return true;
  }

  bool ElevationMapping::disableUpdatesServiceCallback(const std::shared_ptr<rmw_request_id_t>,
                                                       const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                       std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(nodeHandle_->get_logger(), "Disabling updates.");
    updatesEnabled_ = false;
    return true;
  }

  bool ElevationMapping::enableUpdatesServiceCallback(const std::shared_ptr<rmw_request_id_t>,
                                                      const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                      std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(nodeHandle_->get_logger(), "Enabling updates.");
    updatesEnabled_ = true;
    return true;
  }

  bool ElevationMapping::initializeElevationMap()
  {
    geometry_msgs::msg::TransformStamped transform_msg;
    tf2::Stamped<tf2::Transform> transform;

    // Listen to transform between mapFrameId_ and targetFrameInitSubmap_ and use z value for initialization
    try
    {
      transform_msg = transformBuffer_->lookupTransform(mapFrameId_, targetFrameInitSubmap_, tf2::TimePointZero);
      tf2::fromMsg(transform_msg, transform);

      RCLCPP_DEBUG_STREAM(nodeHandle_->get_logger(), "Initializing with x: " << transform.getOrigin().x() << " y: " << transform.getOrigin().y()
                                                                             << " z: " << transform.getOrigin().z());

      const grid_map::Position positionRobot(transform.getOrigin().x(), transform.getOrigin().y());

      // Move map before we apply the height values. This prevents unwanted behavior from intermediate move() calls in
      // updateMapLocation().
      map_.move(positionRobot);

      map_.setRawSubmapHeight(positionRobot, transform.getOrigin().z(), lengthInXInitSubmap_,
                              lengthInYInitSubmap_, marginInitSubmap_);
      return true;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_DEBUG(nodeHandle_->get_logger(), "%s", ex.what());
      RCLCPP_WARN(nodeHandle_->get_logger(), "Could not initialize elevation map with constant height. (This warning can be ignored if TF tree is not available.)");
      return false;
    }

    return true;
  }

  bool ElevationMapping::clearMapServiceCallback(const std::shared_ptr<rmw_request_id_t>,
                                                 const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                 std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(nodeHandle_->get_logger(), "Clearing map...");
    bool success = map_.clear();
    success &= initializeElevationMap();
    RCLCPP_INFO(nodeHandle_->get_logger(), "Map cleared.");

    return success;
  }

  bool ElevationMapping::maskedReplaceServiceCallback(std::shared_ptr<grid_map_msgs::srv::SetGridMap::Request> request,
                                                      std::shared_ptr<grid_map_msgs::srv::SetGridMap::Response> /*response*/)
  {
    RCLCPP_INFO(nodeHandle_->get_logger(), "Masked replacing of map.");
    grid_map::GridMap sourceMap;
    grid_map::GridMapRosConverter::fromMessage(request->map, sourceMap);

    // Use the supplied mask or do not use a mask
    grid_map::Matrix mask;
    if (sourceMap.exists("mask"))
    {
      mask = sourceMap["mask"];
    }
    else
    {
      mask = Eigen::MatrixXf::Ones(sourceMap.getSize()(0), sourceMap.getSize()(1));
    }

    std::unique_lock<std::recursive_mutex> scopedLockRawData(map_.getRawDataMutex());

    // Loop over all layers that should be set
    for (auto sourceLayerIterator = sourceMap.getLayers().begin(); sourceLayerIterator != sourceMap.getLayers().end();
         sourceLayerIterator++)
    {
      // skip "mask" layer
      if (*sourceLayerIterator == "mask")
      {
        continue;
      }
      grid_map::Matrix &sourceLayer = sourceMap[*sourceLayerIterator];
      // Check if the layer exists in the elevation map
      if (map_.getRawGridMap().exists(*sourceLayerIterator))
      {
        grid_map::Matrix &destinationLayer = map_.getRawGridMap()[*sourceLayerIterator];
        for (grid_map::GridMapIterator destinationIterator(map_.getRawGridMap()); !destinationIterator.isPastEnd(); ++destinationIterator)
        {
          // Use the position to find corresponding indices in source and destination
          const grid_map::Index destinationIndex(*destinationIterator);
          grid_map::Position position;
          map_.getRawGridMap().getPosition(*destinationIterator, position);

          if (!sourceMap.isInside(position))
          {
            continue;
          }

          grid_map::Index sourceIndex;
          sourceMap.getIndex(position, sourceIndex);
          // If the mask allows it, set the value from source to destination
          if (!std::isnan(mask(sourceIndex(0), sourceIndex(1))))
          {
            destinationLayer(destinationIndex(0), destinationIndex(1)) = sourceLayer(sourceIndex(0), sourceIndex(1));
          }
        }
      }
      else
      {
        RCLCPP_ERROR(nodeHandle_->get_logger(), "Masked replace service: Layer %s does not exist!", sourceLayerIterator->c_str());
      }
    }

    return true;
  }

  bool ElevationMapping::saveMapServiceCallback(std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                                                std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response)
  {
    RCLCPP_INFO(nodeHandle_->get_logger(), "Saving map to file.");
    std::string topic = std::string(nodeHandle_->get_namespace()) + "/elevation_map";
    if (!request->topic_name.empty())
    {
      topic = std::string(nodeHandle_->get_namespace()) + "/" + request->topic_name;
    }
    response->success = static_cast<unsigned char>(
        (grid_map::GridMapRosConverter::saveToBag(map_.getRawGridMap(), request->file_path + "_raw", topic + "_raw")) &&
        static_cast<bool>(response->success));
    return static_cast<bool>(response->success);
  }

  bool ElevationMapping::loadMapServiceCallback(std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                                                std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response)
  {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Loading from bag file.");
    std::unique_lock<std::recursive_mutex> scopedLockRaw(map_.getRawDataMutex());

    std::string topic = nodeHandle_->get_namespace();
    if (!request->topic_name.empty())
    {
      topic += "/" + request->topic_name;
    }
    else
    {
      topic += "/elevation_map";
    }

    response->success = static_cast<unsigned char>(
        grid_map::GridMapRosConverter::loadFromBag(request->file_path + "_raw", topic + "_raw", map_.getRawGridMap()) &&
        static_cast<bool>(response->success));

    // Update timestamp for visualization in ROS
    map_.setTimestamp(nodeHandle_->get_clock()->now());
    map_.publishRawElevationMap();
    return static_cast<bool>(response->success);
  }

} // namespace elevation_mapping
