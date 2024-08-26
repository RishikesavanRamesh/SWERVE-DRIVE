cmake_minimum_required(VERSION 3.8)
project(swervebot_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add swervebot_controller library related compile commands
generate_parameter_library(swervebot_controller_parameters
  src/swervebot_controller.yaml
  include/swervebot_controller/validate_swervebot_controller_parameters.hpp
)
add_library(
  swervebot_controller
  SHARED
  src/swervebot_controller.cpp
)
target_include_directories(swervebot_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(swervebot_controller swervebot_controller_parameters)
ament_target_dependencies(swervebot_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(swervebot_controller PRIVATE "SWERVEBOT_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface swervebot_controller.xml)

install(
  TARGETS
  swervebot_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  
  
  ament_add_gmock(test_load_swervebot_controller test/test_load_swervebot_controller.cpp)
  target_include_directories(test_load_swervebot_controller PRIVATE include)
  ament_target_dependencies(
    test_load_swervebot_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(test_swervebot_controller test/test_swervebot_controller.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/swervebot_controller_params.yaml)
  target_include_directories(test_swervebot_controller PRIVATE include)
  target_link_libraries(test_swervebot_controller swervebot_controller)
  ament_target_dependencies(
    test_swervebot_controller
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(test_swervebot_controller_preceeding test/test_swervebot_controller_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/swervebot_controller_preceeding_params.yaml)
  target_include_directories(test_swervebot_controller_preceeding PRIVATE include)
  target_link_libraries(test_swervebot_controller_preceeding swervebot_controller)
  ament_target_dependencies(
    test_swervebot_controller_preceeding
    controller_interface
    hardware_interface
  )
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  swervebot_controller
)

ament_package()
