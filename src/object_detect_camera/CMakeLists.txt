cmake_minimum_required(VERSION 3.8)
project(object_detect_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(car_interfaces /home/ros2data/car_robot/install)
# find dependencies
find_package(VPI 2.0)
find_package(ament_cmake REQUIRED)
find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(CUDA REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(car_interfaces REQUIRED PATHS ${car_interfaces})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
set(ROS_DISTRO "ROS_${ROS_DISTRO}")
message("detected ROS_DISTRO=${ROS_DISTRO}")

link_directories(/usr/lib/aarch64-linux-gnu/tegra)
set(CMAKE_CXX_STANDARD 14)
include_directories(${CUDA_INCLUDE_DIRS})
add_definitions(-DROS2 -DROS_DISTRO=${ROS_DISTRO})


# sources shared across nodes
set(common_src src/image_converter.cpp src/ros2_definitions.cpp)



add_executable(video_source src/node_video_source.cpp ${common_src})
target_link_libraries(video_source ${catkin_LIBRARIES} jetson-inference)
add_executable(video_output src/node_video_output.cpp ${common_src})
target_link_libraries(video_output ${catkin_LIBRARIES} jetson-inference)
#add_executable(imagenet src/imagenet.cpp ${common_src})
#target_link_libraries(imagenet ${catkin_LIBRARIES} jetson-inference)
add_executable(detectnet src/detectnet.cpp ${common_src})
target_link_libraries(detectnet ${catkin_LIBRARIES} jetson-inference)
#add_executable(tracking src/tracking.cpp src/utils/pid.cpp ${common_src})
#target_link_libraries(tracking ${catkin_LIBRARIES} jetson-inference)



ament_target_dependencies(video_source rclcpp std_msgs sensor_msgs vision_msgs)
ament_target_dependencies(video_output rclcpp std_msgs sensor_msgs vision_msgs)
#ament_target_dependencies(imagenet rclcpp std_msgs sensor_msgs vision_msgs)
ament_target_dependencies(detectnet rclcpp std_msgs sensor_msgs vision_msgs)
#ament_target_dependencies(tracking rclcpp std_msgs sensor_msgs geometry_msgs vision_msgs car_interfaces)


install(TARGETS
  video_source
  video_output
  #imagenet
  detectnet
  #tracking
  
  DESTINATION lib/${PROJECT_NAME}
)

#install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
