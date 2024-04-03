cmake_minimum_required(VERSION 3.14)
project(sine_example)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/sine_example/sine_example.hpp
  include/sine_example/visibility_control.hpp
  src/sine_example.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/sine_example/sine_example_node.hpp
  src/sine_example_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "sine_example::SineExampleNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_sine_example
    test/test_sine_example.cpp
  )
  ament_target_dependencies(test_sine_example ${TEST_SOURCES})
  target_link_libraries(test_sine_example ${PROJECT_NAME})
endif()


ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
