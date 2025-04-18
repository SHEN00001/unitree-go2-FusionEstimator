cmake_minimum_required(VERSION 3.5)
project(fusion_estimator)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

enable_language(C CXX)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(include include/common include/nlohmann)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
#   rosbag2_cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(rosbag2_cpp REQUIRED)
find_package(base_interfaces_demo REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(urdfdom REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

# 获取 src/Estimator 目录下的所有 .cpp
file(GLOB_RECURSE ESTIMATOR_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

# add_executable(fusion_estimator_node src/fusion_estimator_node.cpp)
add_executable(fusion_estimator_node
    src/fusion_estimator_node.cpp
    src/GO2FusionEstimator/SensorBase.cpp
    src/GO2FusionEstimator/Sensor_Legs.cpp
    src/GO2FusionEstimator/Sensor_IMU.cpp
    src/GO2FusionEstimator/Estimator/StateSpaceModel1.c  # 关键：添加C文件
    src/GO2FusionEstimator/Estimator/StateSpaceModel2.c
    src/GO2FusionEstimator/Estimator/Cpp_Estimators/Estimator2001_Kalman.cpp
    src/GO2FusionEstimator/Estimator/Cpp_Estimators/Estimator2002_UnscentedKalmanEstiamtor.cpp
)

# 包含目录
target_include_directories(fusion_estimator_node PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${urdfdom_headers_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
)


ament_target_dependencies(fusion_estimator_node 
    "base_interfaces_demo"
    rclcpp
    sensor_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
    builtin_interfaces 
    ${DEPENDENCY_LIST})

ament_export_dependencies(
  rclcpp
  builtin_interfaces
  nav_msgs
  tf2_geometry_msgs
  sensor_msgs
)


target_link_libraries(fusion_estimator_node
  # 类型支持库必须放在最前面
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
)

# 链接库
target_link_libraries(fusion_estimator_node
  ${ament_LIBRARIES}
  ${cpp_typesupport_target}
  ${urdfdom_LIBRARIES}
)

target_link_libraries(fusion_estimator_node
  builtin_interfaces::builtin_interfaces__rosidl_typesupport_cpp
)



install(TARGETS 
        fusion_estimator_node
        DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
