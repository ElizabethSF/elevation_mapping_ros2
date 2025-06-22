cmake_minimum_required(VERSION 3.8)
project(elevation_mapping_demos)

# Find required ROS 2 and dependent packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(elevation_mapping REQUIRED)
find_package(grid_map_visualization REQUIRED)

# Install configuration, docs, launch, rviz, and sample data
install(DIRECTORY
  config
  doc
  launch
  rviz
  sample_data
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/tf_to_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/empty_test.cpp
  )
endif()

# Package export
ament_package()
