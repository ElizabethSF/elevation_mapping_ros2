/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <cmath>
#include <cstring>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapFunctors.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"

namespace
{
  /**
   * Store an unsigned integer value in a float
   * @param input integer
   * @return A float with the bit pattern of the input integer
   */
  float intAsFloat(const uint32_t input)
  {
    float output;
    std::memcpy(&output, &input, sizeof(uint32_t));
    return output;
  }
} // namespace

namespace elevation_mapping
{

  ElevationMap::ElevationMap(std::shared_ptr<rclcpp::Node> nodeHandle)
      : nodeHandle_(nodeHandle),
        rawMap_({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy", "color", "time",
                 "dynamic_time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan", "sensor_z_at_lowest_scan"})
  {
    rawMap_.setBasicLayers({"elevation", "variance"});
    clear();

    readParameters();
    setupPublishers();

    initialTime_ = nodeHandle_->get_clock()->now();
  }

  ElevationMap::~ElevationMap() = default;

  bool ElevationMap::readParameters()
  {
    elevationMapTopic_ = nodeHandle_->declare_parameter("elevation_map_topic", std::string("/elevation_map"));
    frameId_ = nodeHandle_->declare_parameter("map_frame_id", std::string("map"));
    setFrameId(frameId_);

    grid_map::Length length;
    grid_map::Position position;
    double resolution;

    length(0) = nodeHandle_->declare_parameter("length_in_x", 100.0);
    length(1) = nodeHandle_->declare_parameter("length_in_y", 100.0);
    position.x() = nodeHandle_->declare_parameter("position_x", 0.0);
    position.y() = nodeHandle_->declare_parameter("position_y", 0.0);
    resolution = nodeHandle_->declare_parameter("resolution", 0.2);
    setGeometry(length, resolution, position);

    minVariance_ = nodeHandle_->declare_parameter("min_variance", pow(0.003, 2));
    maxVariance_ = nodeHandle_->declare_parameter("max_variance", pow(0.03, 2));
    mahalanobisDistanceThreshold_ = nodeHandle_->declare_parameter("mahalanobis_distance_threshold", 2.5);
    multiHeightNoise_ = nodeHandle_->declare_parameter("multi_height_noise", pow(0.003, 2));
    minHorizontalVariance_ = nodeHandle_->declare_parameter("min_horizontal_variance", pow(resolution / 2.0, 2)); // two-sigma
    maxHorizontalVariance_ = nodeHandle_->declare_parameter("max_horizontal_variance", 0.5);
    enableVisibilityCleanup_ = nodeHandle_->declare_parameter("enable_visibility_cleanup", true);
    enableContinuousCleanup_ = nodeHandle_->declare_parameter("enable_continuous_cleanup", false);
    scanningDuration_ = nodeHandle_->declare_parameter("scanning_duration", 1.0);

    std::cout << "read params" << std::endl;

    return true;
  }

  void ElevationMap::setupPublishers()
  {
    rawMapPublisher_ = nodeHandle_->create_publisher<grid_map_msgs::msg::GridMap>(elevationMapTopic_, 1);

    if (enableVisibilityCleanup_) {
      visibilityCleanupMapPublisher_ = nodeHandle_->create_publisher<grid_map_msgs::msg::GridMap>("visibility_cleanup_map", 1);
    }
  }

  void ElevationMap::setGeometry(const grid_map::Length &length, const double &resolution, const grid_map::Position &position)
  {
    std::unique_lock<std::recursive_mutex> scopedLockForRawData(rawMapMutex_);
    rawMap_.setGeometry(length, resolution, position);
    RCLCPP_INFO_STREAM(nodeHandle_->get_logger(), "Elevation map grid resized to " << rawMap_.getSize()(0) << " rows and " << rawMap_.getSize()(1) << " columns.");
  }

  bool ElevationMap::add(const PointCloudType::Ptr pointCloud, Eigen::VectorXf &pointCloudVariances, const rclcpp::Time &timestamp,
                         const Eigen::Affine3d &transformationSensorToMap)
  {
    if (static_cast<unsigned int>(pointCloud->size()) != static_cast<unsigned int>(pointCloudVariances.size()))
    {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "ElevationMap::add: Size of point cloud (%i) and variances (%i) do not agree.", (int)pointCloud->size(),
                   (int)pointCloudVariances.size());
      return false;
    }

    // Initialization for time calculation.
    const auto methodStartTime = std::chrono::system_clock::now();
    const rclcpp::Time currentTime = nodeHandle_->get_clock()->now();
    const float currentTimeSecondsPattern{intAsFloat(static_cast<uint32_t>(static_cast<uint64_t>(currentTime.seconds())))};
    std::unique_lock<std::recursive_mutex> scopedLockForRawData(rawMapMutex_);

    // Update initial time if it is not initialized.
    if (initialTime_.seconds() == 0)
    {
      initialTime_ = timestamp;
    }
    const float scanTimeSinceInitialization = (timestamp - initialTime_).seconds();

    // Store references for efficient interation.
    auto &elevationLayer = rawMap_["elevation"];
    auto &varianceLayer = rawMap_["variance"];
    auto &horizontalVarianceXLayer = rawMap_["horizontal_variance_x"];
    auto &horizontalVarianceYLayer = rawMap_["horizontal_variance_y"];
    auto &horizontalVarianceXYLayer = rawMap_["horizontal_variance_xy"];
    auto &colorLayer = rawMap_["color"];
    auto &timeLayer = rawMap_["time"];
    auto &dynamicTimeLayer = rawMap_["dynamic_time"];
    auto &lowestScanPointLayer = rawMap_["lowest_scan_point"];
    auto &sensorXatLowestScanLayer = rawMap_["sensor_x_at_lowest_scan"];
    auto &sensorYatLowestScanLayer = rawMap_["sensor_y_at_lowest_scan"];
    auto &sensorZatLowestScanLayer = rawMap_["sensor_z_at_lowest_scan"];

    std::vector<Eigen::Ref<const grid_map::Matrix>> basicLayers_;
    for (const std::string &layer : rawMap_.getBasicLayers())
    {
      basicLayers_.push_back(rawMap_.get(layer));
    }

    for (unsigned int i = 0; i < pointCloud->size(); ++i)
    {
      auto &point = pointCloud->points[i];
      grid_map::Index index;
      grid_map::Position position(point.x, point.y); // NOLINT(cppcoreguidelines-pro-type-union-access)
      if (!rawMap_.getIndex(position, index))
      {
        continue; // Skip this point if it does not lie within the elevation map.
      }

      auto &elevation = elevationLayer(index(0), index(1));
      auto &variance = varianceLayer(index(0), index(1));
      auto &horizontalVarianceX = horizontalVarianceXLayer(index(0), index(1));
      auto &horizontalVarianceY = horizontalVarianceYLayer(index(0), index(1));
      auto &horizontalVarianceXY = horizontalVarianceXYLayer(index(0), index(1));
      auto &color = colorLayer(index(0), index(1));
      auto &time = timeLayer(index(0), index(1));
      auto &dynamicTime = dynamicTimeLayer(index(0), index(1));
      auto &lowestScanPoint = lowestScanPointLayer(index(0), index(1));
      auto &sensorXatLowestScan = sensorXatLowestScanLayer(index(0), index(1));
      auto &sensorYatLowestScan = sensorYatLowestScanLayer(index(0), index(1));
      auto &sensorZatLowestScan = sensorZatLowestScanLayer(index(0), index(1));

      const float &pointVariance = pointCloudVariances(i);
      bool isValid = std::all_of(basicLayers_.begin(), basicLayers_.end(),
                                 [&](Eigen::Ref<const grid_map::Matrix> layer)
                                 { return std::isfinite(layer(index(0), index(1))); });
      if (!isValid)
      {
        // No prior information in elevation map, use measurement.
        elevation = point.z; // NOLINT(cppcoreguidelines-pro-type-union-access)
        variance = pointVariance;
        horizontalVarianceX = minHorizontalVariance_;
        horizontalVarianceY = minHorizontalVariance_;
        horizontalVarianceXY = 0.0;
        grid_map::colorVectorToValue(point.getRGBVector3i(), color);
        continue;
      }

      // Deal with multiple heights in one cell.
      const double mahalanobisDistance = fabs(point.z - elevation) / sqrt(variance); // NOLINT(cppcoreguidelines-pro-type-union-access)
      if (mahalanobisDistance > mahalanobisDistanceThreshold_)
      {
        if (scanTimeSinceInitialization - time <= scanningDuration_ &&
            elevation > point.z)
        { // NOLINT(cppcoreguidelines-pro-type-union-access)
          // Ignore point if measurement is from the same point cloud (time comparison) and
          // if measurement is lower then the elevation in the map.
        }
        else if (scanTimeSinceInitialization - time <= scanningDuration_)
        {
          // If point is higher.
          elevation = point.z; // NOLINT(cppcoreguidelines-pro-type-union-access)
          variance = pointVariance;
        }
        else
        {
          variance += multiHeightNoise_;
        }
        continue;
      }

      // Store lowest points from scan for visibility checking.
      const float pointHeightPlusUncertainty =
          point.z + 3.0 * sqrt(pointVariance); // 3 sigma. // NOLINT(cppcoreguidelines-pro-type-union-access)
      if (std::isnan(lowestScanPoint) || pointHeightPlusUncertainty < lowestScanPoint)
      {
        lowestScanPoint = pointHeightPlusUncertainty;
        const grid_map::Position3 sensorTranslation(transformationSensorToMap.translation());
        sensorXatLowestScan = sensorTranslation.x();
        sensorYatLowestScan = sensorTranslation.y();
        sensorZatLowestScan = sensorTranslation.z();
      }

      // Fuse measurement with elevation map data.
      elevation =
          (variance * point.z + pointVariance * elevation) / (variance + pointVariance); // NOLINT(cppcoreguidelines-pro-type-union-access)
      variance = (pointVariance * variance) / (pointVariance + variance);
      // TODO(max): Add color fusion.
      grid_map::colorVectorToValue(point.getRGBVector3i(), color);
      time = scanTimeSinceInitialization;
      dynamicTime = currentTimeSecondsPattern;

      // Horizontal variances are reset.
      horizontalVarianceX = minHorizontalVariance_;
      horizontalVarianceY = minHorizontalVariance_;
      horizontalVarianceXY = 0.0;
    }

    clean();
    rawMap_.setTimestamp(timestamp.nanoseconds()); // Point cloud stores time in microseconds.

    const std::chrono::duration<double> duration = std::chrono::system_clock::now() - methodStartTime;
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Raw map has been updated with a new point cloud in %f s.", duration.count());
    return true;
  }

  bool ElevationMap::update(const grid_map::Matrix &varianceUpdate, const grid_map::Matrix &horizontalVarianceUpdateX,
                            const grid_map::Matrix &horizontalVarianceUpdateY, const grid_map::Matrix &horizontalVarianceUpdateXY,
                            const rclcpp::Time &time)
  {
    std::unique_lock<std::recursive_mutex> scopedLock(rawMapMutex_);

    const auto &size = rawMap_.getSize();

    if (!((grid_map::Index(varianceUpdate.rows(), varianceUpdate.cols()) == size).all() &&
          (grid_map::Index(horizontalVarianceUpdateX.rows(), horizontalVarianceUpdateX.cols()) == size).all() &&
          (grid_map::Index(horizontalVarianceUpdateY.rows(), horizontalVarianceUpdateY.cols()) == size).all() &&
          (grid_map::Index(horizontalVarianceUpdateXY.rows(), horizontalVarianceUpdateXY.cols()) == size).all()))
    {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "The size of the update matrices does not match.");
      return false;
    }

    rawMap_.get("variance") += varianceUpdate;
    rawMap_.get("horizontal_variance_x") += horizontalVarianceUpdateX;
    rawMap_.get("horizontal_variance_y") += horizontalVarianceUpdateY;
    rawMap_.get("horizontal_variance_xy") += horizontalVarianceUpdateXY;
    clean();
    rawMap_.setTimestamp(time.nanoseconds());

    return true;
  }

  bool ElevationMap::clear()
  {
    // Lock raw map object in different scopes to prevent deadlock.
    {
      std::unique_lock<std::recursive_mutex> scopedLockForRawData(rawMapMutex_);
      rawMap_.clearAll();
      rawMap_.resetTimestamp();
      rawMap_.get("dynamic_time").setZero();
    }
    return true;
  }

  void ElevationMap::visibilityCleanup(const rclcpp::Time &updatedTime)
  {
    // Get current time to compute calculation time.
    const auto methodStartTime = nodeHandle_->get_clock()->now();
    const double timeSinceInitialization = (updatedTime - initialTime_).seconds();

    // Copy raw elevation map data for safe multi-threading.
    std::unique_lock<std::recursive_mutex> scopedLockForVisibilityCleanupData(visibilityCleanupMapMutex_);
    std::unique_lock<std::recursive_mutex> scopedLockForRawData(rawMapMutex_);
    visibilityCleanupMap_ = rawMap_;
    rawMap_.clear("lowest_scan_point");
    rawMap_.clear("sensor_x_at_lowest_scan");
    rawMap_.clear("sensor_y_at_lowest_scan");
    rawMap_.clear("sensor_z_at_lowest_scan");
    scopedLockForRawData.unlock();
    visibilityCleanupMap_.add("max_height");

    // Create max. height layer with ray tracing.
    for (grid_map::GridMapIterator iterator(visibilityCleanupMap_); !iterator.isPastEnd(); ++iterator)
    {
      if (!visibilityCleanupMap_.isValid(*iterator))
      {
        continue;
      }
      const auto &lowestScanPoint = visibilityCleanupMap_.at("lowest_scan_point", *iterator);
      const auto &sensorXatLowestScan = visibilityCleanupMap_.at("sensor_x_at_lowest_scan", *iterator);
      const auto &sensorYatLowestScan = visibilityCleanupMap_.at("sensor_y_at_lowest_scan", *iterator);
      const auto &sensorZatLowestScan = visibilityCleanupMap_.at("sensor_z_at_lowest_scan", *iterator);
      if (std::isnan(lowestScanPoint))
      {
        continue;
      }
      grid_map::Index indexAtSensor;
      if (!visibilityCleanupMap_.getIndex(grid_map::Position(sensorXatLowestScan, sensorYatLowestScan), indexAtSensor))
      {
        continue;
      }
      grid_map::Position point;
      visibilityCleanupMap_.getPosition(*iterator, point);
      float pointDiffX = point.x() - sensorXatLowestScan;
      float pointDiffY = point.y() - sensorYatLowestScan;
      float distanceToPoint = sqrt(pointDiffX * pointDiffX + pointDiffY * pointDiffY);
      if (distanceToPoint > 0.0)
      {
        for (grid_map::LineIterator iterator(visibilityCleanupMap_, indexAtSensor, *iterator); !iterator.isPastEnd(); ++iterator)
        {
          grid_map::Position cellPosition;
          visibilityCleanupMap_.getPosition(*iterator, cellPosition);
          const float cellDiffX = cellPosition.x() - sensorXatLowestScan;
          const float cellDiffY = cellPosition.y() - sensorYatLowestScan;
          const float distanceToCell = distanceToPoint - sqrt(cellDiffX * cellDiffX + cellDiffY * cellDiffY);
          const float maxHeightPoint = lowestScanPoint + (sensorZatLowestScan - lowestScanPoint) / distanceToPoint * distanceToCell;
          auto &cellMaxHeight = visibilityCleanupMap_.at("max_height", *iterator);
          if (std::isnan(cellMaxHeight) || cellMaxHeight > maxHeightPoint)
          {
            cellMaxHeight = maxHeightPoint;
          }
        }
      }
    }

    // Vector of indices that will be removed.
    std::vector<grid_map::Position> cellPositionsToRemove;
    for (grid_map::GridMapIterator iterator(visibilityCleanupMap_); !iterator.isPastEnd(); ++iterator)
    {
      if (!visibilityCleanupMap_.isValid(*iterator))
      {
        continue;
      }
      const auto &time = visibilityCleanupMap_.at("time", *iterator);
      if (timeSinceInitialization - time > scanningDuration_)
      {
        // Only remove cells that have not been updated during the last scan duration.
        // This prevents a.o. removal of overhanging objects.
        const auto &elevation = visibilityCleanupMap_.at("elevation", *iterator);
        const auto &variance = visibilityCleanupMap_.at("variance", *iterator);
        const auto &maxHeight = visibilityCleanupMap_.at("max_height", *iterator);
        if (!std::isnan(maxHeight) && elevation - 3.0 * sqrt(variance) > maxHeight)
        {
          grid_map::Position position;
          visibilityCleanupMap_.getPosition(*iterator, position);
          cellPositionsToRemove.push_back(position);
        }
      }
    }

    // Remove points in current raw map.
    scopedLockForRawData.lock();
    for (const auto &cellPosition : cellPositionsToRemove)
    {
      grid_map::Index index;
      if (!rawMap_.getIndex(cellPosition, index))
      {
        continue;
      }
      if (rawMap_.isValid(index))
      {
        rawMap_.at("elevation", index) = NAN;
        rawMap_.at("dynamic_time", index) = 0.0f;
      }
    }
    scopedLockForRawData.unlock();

    // Publish visibility cleanup map for debugging.
    publishVisibilityCleanupMap();

    const rclcpp::Duration duration = nodeHandle_->get_clock()->now() - methodStartTime;
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Visibility cleanup has been performed in %f s (%d points).", duration.seconds(), (int)cellPositionsToRemove.size());
    if (duration.seconds() > visibilityCleanupDuration_)
    {
      RCLCPP_WARN(nodeHandle_->get_logger(), "Visibility cleanup duration is too high (current rate is %f).", 1.0 / duration.seconds());
    }
  }

  void ElevationMap::move(const Eigen::Vector2d &position)
  {
    std::unique_lock<std::recursive_mutex> scopedLockForRawData(rawMapMutex_);
    std::vector<grid_map::BufferRegion> newRegions;

    if (rawMap_.move(position, newRegions))
    {
      RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map has been moved to position (%f, %f).", rawMap_.getPosition().x(), rawMap_.getPosition().y());

      // The "dynamic_time" layer is meant to be interpreted as integer values, therefore nan:s need to be zeroed.
      grid_map::Matrix &dynTime{rawMap_.get("dynamic_time")};
      dynTime = dynTime.array().isNaN().select(grid_map::Matrix::Scalar(0.0f), dynTime.array());
    }
  }

  bool ElevationMap::publishRawElevationMap()
  {
    std::unique_lock<std::recursive_mutex> scopedLock(rawMapMutex_);
    grid_map::GridMap rawMapCopy = rawMap_;
    scopedLock.unlock();

    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(rawMapCopy);
    rawMapPublisher_->publish(std::move(message)); // CRASHES HERE

    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map has been published.");
    return true;
  }

  bool ElevationMap::publishVisibilityCleanupMap()
  {
    if (visibilityCleanupMapPublisher_->get_subscription_count() < 1)
    {
      return false;
    }
    
    std::unique_lock<std::recursive_mutex> scopedLock(visibilityCleanupMapMutex_);
    grid_map::GridMap visibilityCleanupMapCopy = visibilityCleanupMap_;
    scopedLock.unlock();

    visibilityCleanupMapCopy.erase("elevation");
    visibilityCleanupMapCopy.erase("variance");
    visibilityCleanupMapCopy.erase("horizontal_variance_x");
    visibilityCleanupMapCopy.erase("horizontal_variance_y");
    visibilityCleanupMapCopy.erase("horizontal_variance_xy");
    visibilityCleanupMapCopy.erase("color");
    visibilityCleanupMapCopy.erase("time");

    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(visibilityCleanupMapCopy);
    visibilityCleanupMapPublisher_->publish(std::move(message));

    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Visibility cleanup map has been published.");
    return true;
  }

  grid_map::GridMap &ElevationMap::getRawGridMap()
  {
    return rawMap_;
  }

  void ElevationMap::setRawGridMap(const grid_map::GridMap &map)
  {
    std::unique_lock<std::recursive_mutex> scopedLockForRawData(rawMapMutex_);
    rawMap_ = map;
  }

  rclcpp::Time ElevationMap::getTimeOfLastUpdate()
  {
    return rclcpp::Time(rawMap_.getTimestamp(), RCL_ROS_TIME);
  }

  const kindr::HomTransformQuatD &ElevationMap::getPose()
  {
    return pose_;
  }

  bool ElevationMap::getPosition3dInRobotParentFrame(const Eigen::Array2i &index, kindr::Position3D &position)
  {
    kindr::Position3D positionInGridFrame;
    if (!rawMap_.getPosition3("elevation", index, positionInGridFrame.vector()))
    {
      return false;
    }
    position = pose_.transform(positionInGridFrame);
    return true;
  }

  std::recursive_mutex &ElevationMap::getRawDataMutex()
  {
    return rawMapMutex_;
  }

  bool ElevationMap::clean()
  {
    std::unique_lock<std::recursive_mutex> scopedLockForRawData(rawMapMutex_);
    rawMap_.get("variance") = rawMap_.get("variance").unaryExpr(VarianceClampOperator<float>(minVariance_, maxVariance_));
    rawMap_.get("horizontal_variance_x") =
        rawMap_.get("horizontal_variance_x").unaryExpr(VarianceClampOperator<float>(minHorizontalVariance_, maxHorizontalVariance_));
    rawMap_.get("horizontal_variance_y") =
        rawMap_.get("horizontal_variance_y").unaryExpr(VarianceClampOperator<float>(minHorizontalVariance_, maxHorizontalVariance_));
    return true;
  }

  void ElevationMap::setFrameId(const std::string &frameId)
  {
    rawMap_.setFrameId(frameId);
  }

  void ElevationMap::setTimestamp(rclcpp::Time timestamp)
  {
    rawMap_.setTimestamp(timestamp.nanoseconds());
  }

  const std::string &ElevationMap::getFrameId()
  {
    return rawMap_.getFrameId();
  }

  void ElevationMap::setRawSubmapHeight(const grid_map::Position &initPosition, float mapHeight, double lengthInXSubmap,
                                        double lengthInYSubmap, double margin)
  {
    // Set a submap area (lengthInYSubmap + margin, lengthInXSubmap + margin) with a constant height (mapHeight).
    std::unique_lock<std::recursive_mutex> scopedLockForRawData(rawMapMutex_);

    // Calculate submap iterator start index.
    const grid_map::Position topLeftPosition(initPosition(0) + lengthInXSubmap / 2, initPosition(1) + lengthInYSubmap / 2);
    grid_map::Index submapTopLeftIndex;
    rawMap_.getIndex(topLeftPosition, submapTopLeftIndex);

    // Calculate submap area.
    const double resolution = rawMap_.getResolution();
    const int lengthInXSubmapI = static_cast<int>(lengthInXSubmap / resolution + 2 * margin);
    const int lengthInYSubmapI = static_cast<int>(lengthInYSubmap / resolution + 2 * margin);
    const Eigen::Array2i submapBufferSize(lengthInYSubmapI, lengthInXSubmapI);

    // Iterate through submap and fill height values.
    grid_map::Matrix &elevationData = rawMap_["elevation"];
    grid_map::Matrix &varianceData = rawMap_["variance"];
    for (grid_map::SubmapIterator iterator(rawMap_, submapTopLeftIndex, submapBufferSize); !iterator.isPastEnd(); ++iterator)
    {
      const grid_map::Index index(*iterator);
      elevationData(index(0), index(1)) = mapHeight;
      varianceData(index(0), index(1)) = 0.0;
    }
  }

  float ElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation)
  {
    return 0.5 * erfc(-(x - mean) / (standardDeviation * sqrt(2.0)));
  }

} // namespace elevation_mapping
