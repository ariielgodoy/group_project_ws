cmake_minimum_required(VERSION 3.8)

project(rosgame_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Uncomment the following section to manually fill in further dependencies.
# find_package(<dependency> REQUIRED)

# Enable linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the linter that checks for copyrights.
  # Comment out this line when copyrights and licenses are added to all source files.
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (works only in a git repo).
  # Comment out this line when this package is in a git repo and when
  # copyrights and licenses are added to all source files.
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Find required dependencies
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgame_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate ROS messages interfaces
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RosgameRegister.srv"
)

include_directories(include)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        )
        
# Define the "bridge" executable
add_executable(bridge src/bridge.cpp src/main_bridge.cpp)

# Get ROSIDL typesupport target
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# Link the "bridge" executable with ROSIDL typesupport
target_link_libraries(bridge "${cpp_typesupport_target}")

# Find and include JSONCPP package
find_package(jsoncpp REQUIRED)
include_directories(${jsoncpp_INCLUDE_DIRS})

# Link the "bridge" executable with JSONCPP
target_link_libraries(bridge jsoncpp)

# Install the "bridge" executable
install(TARGETS
  bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Ament target dependencies
ament_target_dependencies(bridge
                          rosgame_msgs
                          rclcpp
                          std_msgs
                          geometry_msgs
                          sensor_msgs)

ament_package()
