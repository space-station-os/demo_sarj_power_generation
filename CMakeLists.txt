cmake_minimum_required(VERSION 3.8)
project(demo_sarj_power_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(power_generation src/power_generation.cpp)
add_executable(select_sarj_angle src/select_sarj_angle.cpp)
ament_target_dependencies(
  power_generation 
  rclcpp 
  geometry_msgs
  std_msgs
  )
ament_target_dependencies(
  select_sarj_angle 
  rclcpp 
  geometry_msgs
  std_msgs
  )

install(TARGETS
  power_generation
  select_sarj_angle
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(power_generation PRIVATE ${EIGEN3_INCLUDE_DIR})
target_include_directories(select_sarj_angle PRIVATE ${EIGEN3_INCLUDE_DIR})

ament_package()
