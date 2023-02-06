cmake_minimum_required(VERSION 3.8)
project(car_motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(car_interfaces /home/ros2data/car_robot/install)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(car_interfaces REQUIRED PATHS ${car_interfaces} )
find_package(JetsonGPIO REQUIRED)

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

add_executable(motorcontroller src/motor_controller.cpp 
                           drivers/AdafruitMotorHatLibrary/adafruitmotorhat.o
                           drivers/AdafruitMotorHatLibrary/adafruitdcmotor.o
                           drivers/AdafruitMotorHatLibrary/i2cdevice.o
                           drivers/AdafruitMotorHatLibrary/pwm.o

)
add_executable(relativespeeds src/relative_speeds.cpp)
add_executable(motorencoder src/motor_encoder.cpp)
add_executable(carpose src/car_pose.cpp)
add_executable(caracctionserver src/car_action_server.cpp)
# add_executable(actionclient src/action_client.cpp)




ament_target_dependencies(motorcontroller
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
)

ament_target_dependencies(relativespeeds
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
)
ament_target_dependencies(motorencoder
  "rclcpp"
  "std_srvs"
  "JetsonGPIO"
  "car_interfaces"
)

ament_target_dependencies(carpose
  "rclcpp"
  "std_msgs"
  "car_interfaces"
)

ament_target_dependencies(caracctionserver
  "rclcpp"
  "car_interfaces"
  "geometry_msgs"
  "std_srvs"
  "std_msgs"
  "rclcpp_action"
)
# ament_target_dependencies(actionclient
#   "rclcpp"
#   "rclcpp_action"
#   "car_interfaces"
# )

 


install(TARGETS

  motorcontroller
  relativespeeds
  motorencoder
  carpose
  caracctionserver
  # actionclient
  
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
