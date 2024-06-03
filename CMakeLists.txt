cmake_minimum_required(VERSION 3.8)
project(ros2_px4_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

find_package(ament_cmake REQUIRED)

find_package(px4_ros2_cpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

file(GLOB MODE_FILES src/mode/*.cpp)

add_executable(modes_runner src/modes_runner.cpp ${MODE_FILES})
target_include_directories(modes_runner PUBLIC include)
ament_target_dependencies(modes_runner px4_ros2_cpp std_msgs)

install(TARGETS
  modes_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
