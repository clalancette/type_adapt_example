cmake_minimum_required(VERSION 3.8)
project(type_adapt_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

add_library(image_sub_no_type_adapt_node SHARED
  src/image_sub_no_type_adapt_node.cpp
)
target_compile_features(image_sub_no_type_adapt_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(image_sub_no_type_adapt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(image_sub_no_type_adapt_node
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(image_sub_no_type_adapt_node PRIVATE "TYPE_ADAPT_EXAMPLE_BUILDING_LIBRARY")
rclcpp_components_register_node(image_sub_no_type_adapt_node
  PLUGIN "type_adapt_example::ImageSubNoTypeAdaptNode"
  EXECUTABLE image_sub_no_type_adapt
)

add_library(image_pub_no_type_adapt_node SHARED
  src/image_pub_no_type_adapt_node.cpp
)
target_compile_features(image_pub_no_type_adapt_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(image_pub_no_type_adapt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(image_pub_no_type_adapt_node
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(image_pub_no_type_adapt_node PRIVATE "TYPE_ADAPT_EXAMPLE_BUILDING_LIBRARY")
rclcpp_components_register_node(image_pub_no_type_adapt_node
  PLUGIN "type_adapt_example::ImagePubNoTypeAdaptNode"
  EXECUTABLE image_pub_no_type_adapt
)

add_library(image_sub_type_adapt_node SHARED
  src/cv_mat_sensor_msgs_image_type_adapter.cpp
  src/image_sub_type_adapt_node.cpp
)
target_compile_features(image_sub_type_adapt_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(image_sub_type_adapt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(image_sub_type_adapt_node
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(image_sub_type_adapt_node PRIVATE "TYPE_ADAPT_EXAMPLE_BUILDING_LIBRARY")
rclcpp_components_register_node(image_sub_type_adapt_node
  PLUGIN "type_adapt_example::ImageSubTypeAdaptNode"
  EXECUTABLE image_sub_type_adapt
)

add_library(image_pub_type_adapt_node SHARED
  src/cv_mat_sensor_msgs_image_type_adapter.cpp
  src/image_pub_type_adapt_node.cpp
)
target_compile_features(image_pub_type_adapt_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(image_pub_type_adapt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(image_pub_type_adapt_node
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(image_pub_type_adapt_node PRIVATE "TYPE_ADAPT_EXAMPLE_BUILDING_LIBRARY")
rclcpp_components_register_node(image_pub_type_adapt_node
  PLUGIN "type_adapt_example::ImagePubTypeAdaptNode"
  EXECUTABLE image_pub_type_adapt
)

add_library(image_sub_type_adapt_intra_node SHARED
  src/cv_mat_sensor_msgs_image_type_adapter.cpp
  src/image_sub_type_adapt_intra_node.cpp
)
target_compile_features(image_sub_type_adapt_intra_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(image_sub_type_adapt_intra_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(image_sub_type_adapt_intra_node
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(image_sub_type_adapt_intra_node PRIVATE "TYPE_ADAPT_EXAMPLE_BUILDING_LIBRARY")
rclcpp_components_register_node(image_sub_type_adapt_intra_node
  PLUGIN "type_adapt_example::ImageSubTypeAdaptIntraNode"
  EXECUTABLE image_sub_type_adapt_intra
)

add_library(image_pub_type_adapt_intra_node SHARED
  src/cv_mat_sensor_msgs_image_type_adapter.cpp
  src/image_pub_type_adapt_intra_node.cpp
)
target_compile_features(image_pub_type_adapt_intra_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(image_pub_type_adapt_intra_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(image_pub_type_adapt_intra_node
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(image_pub_type_adapt_intra_node PRIVATE "TYPE_ADAPT_EXAMPLE_BUILDING_LIBRARY")
rclcpp_components_register_node(image_pub_type_adapt_intra_node
  PLUGIN "type_adapt_example::ImagePubTypeAdaptIntraNode"
  EXECUTABLE image_pub_type_adapt_intra
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS
    image_sub_no_type_adapt_node
    image_pub_no_type_adapt_node
    image_sub_type_adapt_node
    image_pub_type_adapt_node
    image_sub_type_adapt_intra_node
    image_pub_type_adapt_intra_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include/${PROJECT_NAME}
)
ament_export_libraries(
  image_sub_no_type_adapt_node
  image_pub_no_type_adapt_node
  image_sub_type_adapt_node
  image_pub_type_adapt_node
  image_sub_type_adapt_intra_node
  image_pub_type_adapt_intra_node
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
