cmake_minimum_required(VERSION 3.8)
project(liloc)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(ndt_omp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudInfo.msg"
  "srv/SaveMap.srv"
  "srv/SaveSession.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  lib
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  lib
  include
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

file(GLOB common_lib "lib/*.cpp")

set(LILOC_DEPENDENCIES
  rclcpp
  cv_bridge
  PCL
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  tf2_sensor_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  OpenCV
  GTSAM
  Boost
  # ${OpenMP_CXX_FLAGS}
)

# Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp ${common_lib})
# target_compile_options(${PROJECT_NAME}_imageProjection PRIVATE ${OpenMP_CXX_FLAGS})
ament_target_dependencies(${PROJECT_NAME}_imageProjection ${LILOC_DEPENDENCIES} livox_ros_driver2)
# target_link_libraries(${PROJECT_NAME}_imageProjection ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam "${cpp_typesupport_target}")

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptimization src/mapOptimization.cpp ${common_lib})
# target_compile_options(${PROJECT_NAME}_mapOptimization PRIVATE ${OpenMP_CXX_FLAGS})
ament_target_dependencies(${PROJECT_NAME}_mapOptimization ${LILOC_DEPENDENCIES} ndt_omp rosidl_default_runtime)
# target_link_libraries(${PROJECT_NAME}_mapOptimization Boost::timer ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam stdc++fs)

# # IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp ${common_lib})
# target_compile_options(${PROJECT_NAME}_imuPreintegration PRIVATE ${OpenMP_CXX_FLAGS})
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ${LILOC_DEPENDENCIES})
# target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  message("OpenMP found — enabling parallel support")
  target_compile_options(${PROJECT_NAME}_imageProjection PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(${PROJECT_NAME}_imageProjection gtsam "${cpp_typesupport_target}" ${OpenMP_CXX_LIBRARIES})

  target_compile_options(${PROJECT_NAME}_mapOptimization PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam ndt_omp::ndt_omp "${cpp_typesupport_target}" ${OpenMP_CXX_LIBRARIES})

  target_compile_options(${PROJECT_NAME}_imuPreintegration PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer gtsam "${cpp_typesupport_target}" ${OpenMP_CXX_LIBRARIES})
else()
  message(WARNING "OpenMP NOT found - cannot enable parallel support")
  target_link_libraries(${PROJECT_NAME}_imageProjection gtsam "${cpp_typesupport_target}")
  target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam ndt_omp::ndt_omp "${cpp_typesupport_target}")
  target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer gtsam "${cpp_typesupport_target}")
endif()

install(TARGETS
  ${PROJECT_NAME}_imageProjection
  ${PROJECT_NAME}_mapOptimization
  ${PROJECT_NAME}_imuPreintegration
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

#  ${LILOC_DEPENDENCIES})
# ament_export_include_directories(include)
ament_package()
