cmake_minimum_required(VERSION 3.0.2)
project(raicom_mission)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  # complete_mission
  # quadrotor_msgs
  cv_bridge  
  image_transport
  yolov8_ros_msgs
  message_filters
)

# 寻找 PCL 包
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
# 使用 pkg-config 查找 Tesseract 和 Leptonica
find_package(PkgConfig REQUIRED)
pkg_check_modules(TESSERACT REQUIRED tesseract)
pkg_check_modules(LEPTONICA REQUIRED lept)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    pcl_conversions 
    pcl_ros
    # complete_mission
    # quadrotor_msgs
    yolov8_ros_msgs
    message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}  
  ${OpenCV_INCLUDE_DIRS}
  ${TESSERACT_INCLUDE_DIRS}
  ${LEPTONICA_INCLUDE_DIRS}
)

add_executable(scene_generator_node src/scene_generator.cpp)
target_link_libraries(scene_generator_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(raicom_mission src/raicom_mission.cpp)
target_link_libraries(raicom_mission
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(yolo_processor_node src/yolo_processor.cpp)
target_link_libraries(yolo_processor_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 地图累积器节点
add_executable(map_accumulator_node
  src/map_accumulator_node.cpp
)

target_link_libraries(map_accumulator_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(map_accumulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


