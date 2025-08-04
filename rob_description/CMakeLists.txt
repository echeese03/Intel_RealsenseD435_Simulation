cmake_minimum_required(VERSION 3.8)
project(rob_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(realsense_d435 REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)

include_directories(include)

ign_find_package( ignition-plugin1 REQUIRED COMPONENTS register )
ign_find_package( ignition-gazebo6 REQUIRED )

install( DIRECTORY urdf DESTINATION share/${PROJECT_NAME} )
install( DIRECTORY launch DESTINATION share/${PROJECT_NAME} )
install( DIRECTORY rviz DESTINATION share/${PROJECT_NAME} )
install( DIRECTORY world DESTINATION share/${PROJECT_NAME} )

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

ament_package()
