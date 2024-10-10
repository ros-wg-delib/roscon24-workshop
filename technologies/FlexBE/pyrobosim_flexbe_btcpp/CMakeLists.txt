cmake_minimum_required(VERSION 3.8)
project(pyrobosim_flexbe_btcpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPS
  ament_index_cpp
  behaviortree_ros2
  pyrobosim_msgs
  pyrobosim_btcpp
  rclcpp
)

# for each dependency, find and include the package
foreach(DEPENDENCY ${THIS_PACKAGE_DEPS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# create executable
add_executable(flexbe_btcpp_executor src/flexbe_btcpp_executor.cpp)

ament_target_dependencies(flexbe_btcpp_executor ${THIS_PACKAGE_DEPS})

target_include_directories(flexbe_btcpp_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)


# Install executable and trees
install(
  DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS flexbe_btcpp_executor
  DESTINATION DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
    trees
    config
    launch
    DESTINATION share/${PROJECT_NAME}/
    )

ament_package()
