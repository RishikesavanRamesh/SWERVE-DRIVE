cmake_minimum_required(VERSION 3.8)
project(swerve_drive_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  include_directories(include/${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_3_modules_test test/3_modules_test.cpp)
  target_include_directories(${PROJECT_NAME}_3_modules_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_add_gtest(${PROJECT_NAME}_4_modules_test test/4_modules_test.cpp)
  target_include_directories(${PROJECT_NAME}_4_modules_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

endif()


ament_package()
