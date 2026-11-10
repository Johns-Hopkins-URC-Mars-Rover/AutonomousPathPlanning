cmake_minimum_required(VERSION 3.8)
project(rover_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

option(BUILD_ROVER_TESTING "Build rover launch regression tests" ${BUILD_TESTING})
if(BUILD_ROVER_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME rover_simple_switches
    COMMAND "${Python3_EXECUTABLE}" -B -m unittest discover
      -s "${CMAKE_CURRENT_SOURCE_DIR}/test" -p "test_simple_switches.py")
endif()

ament_package()
