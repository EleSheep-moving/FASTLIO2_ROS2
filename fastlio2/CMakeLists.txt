cmake_minimum_required(VERSION 3.8)
project(fastlio2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

option(MP_EN "Enable OpenMP point matching, including on ARM" ON)
set(MP_PROC_NUM "2" CACHE STRING "Number of OpenMP point-matching threads")
if(MP_EN AND NOT MP_PROC_NUM MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "MP_PROC_NUM must be a positive integer")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)

if(MP_EN)
  find_package(OpenMP REQUIRED COMPONENTS CXX)
endif()
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(yaml-cpp REQUIRED)


set(SRC_LIST src/map_builder/commons.cpp
             src/map_builder/ieskf.cpp
             src/map_builder/imu_processor.cpp
             src/map_builder/ikd_Tree.cpp
             src/map_builder/lidar_processor.cpp
             src/map_builder/map_builder.cpp
             src/utils.cpp)

add_executable(lio_node src/lio_node.cpp ${SRC_LIST})
ament_target_dependencies(lio_node rclcpp rcl_interfaces std_msgs tf2 tf2_ros nav_msgs sensor_msgs livox_ros_driver2 pcl_conversions geometry_msgs)
target_link_libraries(lio_node
  yaml-cpp
  ${PCL_LIBRARIES}
)
if(MP_EN)
  target_compile_definitions(lio_node PRIVATE MP_EN MP_PROC_NUM=${MP_PROC_NUM})
  target_link_libraries(lio_node OpenMP::OpenMP_CXX)
  message(STATUS "FAST-LIO OpenMP: ON (${MP_PROC_NUM} threads)")
else()
  message(STATUS "FAST-LIO OpenMP: OFF")
endif()


install(TARGETS lio_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})




if(BUILD_TESTING)
  find_package(Threads REQUIRED)
  add_executable(test_timing_trace test/test_timing_trace.cpp)
  target_include_directories(test_timing_trace PRIVATE src)
  target_link_libraries(test_timing_trace Threads::Threads)
  add_test(NAME test_timing_trace COMMAND test_timing_trace)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu_deskew test/test_imu_deskew.cpp
    src/map_builder/imu_processor.cpp src/map_builder/ieskf.cpp)
  target_include_directories(test_imu_deskew PRIVATE src ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_imu_deskew ${PCL_LIBRARIES})
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
