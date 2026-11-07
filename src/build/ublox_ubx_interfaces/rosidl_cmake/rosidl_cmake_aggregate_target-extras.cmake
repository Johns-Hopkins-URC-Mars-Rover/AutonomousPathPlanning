# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target ublox_ubx_interfaces::ublox_ubx_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${ublox_ubx_interfaces_TARGETS}.
if(ublox_ubx_interfaces_TARGETS AND NOT TARGET ublox_ubx_interfaces::ublox_ubx_interfaces)
  add_library(ublox_ubx_interfaces::ublox_ubx_interfaces INTERFACE IMPORTED)
  set_target_properties(ublox_ubx_interfaces::ublox_ubx_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${ublox_ubx_interfaces_TARGETS}")
endif()
