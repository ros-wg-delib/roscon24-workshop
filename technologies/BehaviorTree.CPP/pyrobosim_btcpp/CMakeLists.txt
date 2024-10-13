cmake_minimum_required(VERSION 3.8)
project(pyrobosim_btcpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPS
  ament_index_cpp
  behaviortree_ros2
  pyrobosim_msgs
  rclcpp
)

# for each dependency, find and include the package
foreach(DEPENDENCY ${THIS_PACKAGE_DEPS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# create executable
add_executable(btcpp_executor src/btcpp_executor.cpp)

ament_target_dependencies(btcpp_executor ${THIS_PACKAGE_DEPS})

target_include_directories(btcpp_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty>)


# Install executable and trees
install(
  DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS btcpp_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(${THIS_PACKAGE_DEPS})
ament_export_include_directories(include)

ament_package()
