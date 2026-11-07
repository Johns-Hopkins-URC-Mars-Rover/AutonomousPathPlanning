# generated from
# ament_cmake_core/cmake/symlink_install/ament_cmake_symlink_install.cmake.in

# create empty symlink install manifest before starting install step
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/symlink_install_manifest.txt")

#
# Reimplement CMake install(DIRECTORY) command to use symlinks instead of
# copying resources.
#
# :param cmake_current_source_dir: The CMAKE_CURRENT_SOURCE_DIR when install
#   was invoked
# :type cmake_current_source_dir: string
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_directory cmake_current_source_dir)
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION" "DIRECTORY;PATTERN;PATTERN_EXCLUDE" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_directory() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # make destination absolute path and ensure that it exists
  if(NOT IS_ABSOLUTE "${ARG_DESTINATION}")
    set(ARG_DESTINATION "/home/clara/AutonomousPathPlanning/src/install/ublox_ubx_msgs/${ARG_DESTINATION}")
  endif()
  if(NOT EXISTS "${ARG_DESTINATION}")
    file(MAKE_DIRECTORY "${ARG_DESTINATION}")
  endif()

  # default pattern to include
  if(NOT ARG_PATTERN)
    set(ARG_PATTERN "*")
  endif()

  # iterate over directories
  foreach(dir ${ARG_DIRECTORY})
    # make dir an absolute path
    if(NOT IS_ABSOLUTE "${dir}")
      set(dir "${cmake_current_source_dir}/${dir}")
    endif()

    if(EXISTS "${dir}")
      # if directory has no trailing slash
      # append folder name to destination
      set(destination "${ARG_DESTINATION}")
      string(LENGTH "${dir}" length)
      math(EXPR offset "${length} - 1")
      string(SUBSTRING "${dir}" ${offset} 1 dir_last_char)
      if(NOT dir_last_char STREQUAL "/")
        get_filename_component(destination_name "${dir}" NAME)
        set(destination "${destination}/${destination_name}")
      else()
        # remove trailing slash
        string(SUBSTRING "${dir}" 0 ${offset} dir)
      endif()
      
      # Create destination directory.
      # This does *not* solve the problem of empty directories WITHIN the install tree,
      # but does make sure that the top-level directory specified by the caller gets created.
      file(MAKE_DIRECTORY "${destination}")

      # glob recursive files
      set(relative_files "")
      foreach(pattern ${ARG_PATTERN})
        file(
          GLOB_RECURSE
          include_files
          RELATIVE "${dir}"
          "${dir}/${pattern}"
        )
        if(NOT include_files STREQUAL "")
          list(APPEND relative_files ${include_files})
        endif()
      endforeach()
      foreach(pattern ${ARG_PATTERN_EXCLUDE})
        file(
          GLOB_RECURSE
          exclude_files
          RELATIVE "${dir}"
          "${dir}/${pattern}"
        )
        if(NOT exclude_files STREQUAL "")
          list(REMOVE_ITEM relative_files ${exclude_files})
        endif()
      endforeach()
      list(SORT relative_files)

      foreach(relative_file ${relative_files})
        set(absolute_file "${dir}/${relative_file}")
        # determine link name for file including destination path
        set(symlink "${destination}/${relative_file}")

        # ensure that destination exists
        get_filename_component(symlink_dir "${symlink}" PATH)
        if(NOT EXISTS "${symlink_dir}")
          file(MAKE_DIRECTORY "${symlink_dir}")
        endif()

        _ament_cmake_symlink_install_create_symlink("${absolute_file}" "${symlink}")
      endforeach()
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_directory() can't find '${dir}'")
      endif()
    endif()
  endforeach()
endfunction()

#
# Reimplement CMake install(FILES) command to use symlinks instead of copying
# resources.
#
# :param cmake_current_source_dir: The CMAKE_CURRENT_SOURCE_DIR when install
#   was invoked
# :type cmake_current_source_dir: string
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_files cmake_current_source_dir)
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION;RENAME" "FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_files() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # make destination an absolute path and ensure that it exists
  if(NOT IS_ABSOLUTE "${ARG_DESTINATION}")
    set(ARG_DESTINATION "/home/clara/AutonomousPathPlanning/src/install/ublox_ubx_msgs/${ARG_DESTINATION}")
  endif()
  if(NOT EXISTS "${ARG_DESTINATION}")
    file(MAKE_DIRECTORY "${ARG_DESTINATION}")
  endif()

  if(ARG_RENAME)
    list(LENGTH ARG_FILES file_count)
    if(NOT file_count EQUAL 1)
    message(FATAL_ERROR "ament_cmake_symlink_install_files() called with "
      "RENAME argument but not with a single file")
    endif()
  endif()

  # iterate over files
  foreach(file ${ARG_FILES})
    # make file an absolute path
    if(NOT IS_ABSOLUTE "${file}")
      set(file "${cmake_current_source_dir}/${file}")
    endif()

    if(EXISTS "${file}")
      # determine link name for file including destination path
      get_filename_component(filename "${file}" NAME)
      if(NOT ARG_RENAME)
        set(symlink "${ARG_DESTINATION}/${filename}")
      else()
        set(symlink "${ARG_DESTINATION}/${ARG_RENAME}")
      endif()
      _ament_cmake_symlink_install_create_symlink("${file}" "${symlink}")
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_files() can't find '${file}'")
      endif()
    endif()
  endforeach()
endfunction()

#
# Reimplement CMake install(PROGRAMS) command to use symlinks instead of copying
# resources.
#
# :param cmake_current_source_dir: The CMAKE_CURRENT_SOURCE_DIR when install
#   was invoked
# :type cmake_current_source_dir: string
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_programs cmake_current_source_dir)
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION" "PROGRAMS" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_programs() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # make destination an absolute path and ensure that it exists
  if(NOT IS_ABSOLUTE "${ARG_DESTINATION}")
    set(ARG_DESTINATION "/home/clara/AutonomousPathPlanning/src/install/ublox_ubx_msgs/${ARG_DESTINATION}")
  endif()
  if(NOT EXISTS "${ARG_DESTINATION}")
    file(MAKE_DIRECTORY "${ARG_DESTINATION}")
  endif()

  # iterate over programs
  foreach(file ${ARG_PROGRAMS})
    # make file an absolute path
    if(NOT IS_ABSOLUTE "${file}")
      set(file "${cmake_current_source_dir}/${file}")
    endif()

    if(EXISTS "${file}")
      # determine link name for file including destination path
      get_filename_component(filename "${file}" NAME)
      set(symlink "${ARG_DESTINATION}/${filename}")
      _ament_cmake_symlink_install_create_symlink("${file}" "${symlink}")
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_programs() can't find '${file}'")
      endif()
    endif()
  endforeach()
endfunction()

#
# Reimplement CMake install(TARGETS) command to use symlinks instead of copying
# resources.
#
# :param TARGET_FILES: the absolute files, replacing the name of targets passed
#   in as TARGETS
# :type TARGET_FILES: list of files
# :param ARGN: the same arguments as the CMake install command except that
#   keywords identifying the kind of type and the DESTINATION keyword must be
#   joined with an underscore, e.g. ARCHIVE_DESTINATION.
# :type ARGN: various
#
function(ament_cmake_symlink_install_targets)
  cmake_parse_arguments(ARG "OPTIONAL" "ARCHIVE_DESTINATION;DESTINATION;LIBRARY_DESTINATION;RUNTIME_DESTINATION"
    "TARGETS;TARGET_FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_targets() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # iterate over target files
  foreach(file ${ARG_TARGET_FILES})
    if(NOT IS_ABSOLUTE "${file}")
      message(FATAL_ERROR "ament_cmake_symlink_install_targets() target file "
        "'${file}' must be an absolute path")
    endif()

    # determine destination of file based on extension
    set(destination "")
    get_filename_component(fileext "${file}" EXT)
    if(fileext STREQUAL ".a" OR fileext STREQUAL ".lib")
      set(destination "${ARG_ARCHIVE_DESTINATION}")
    elseif(fileext STREQUAL ".dylib" OR fileext MATCHES "\\.so(\\.[0-9]+)?(\\.[0-9]+)?(\\.[0-9]+)?$")
      set(destination "${ARG_LIBRARY_DESTINATION}")
    elseif(fileext STREQUAL "" OR fileext STREQUAL ".dll" OR fileext STREQUAL ".exe")
      set(destination "${ARG_RUNTIME_DESTINATION}")
    endif()
    if(destination STREQUAL "")
      set(destination "${ARG_DESTINATION}")
    endif()

    # make destination an absolute path and ensure that it exists
    if(NOT IS_ABSOLUTE "${destination}")
      set(destination "/home/clara/AutonomousPathPlanning/src/install/ublox_ubx_msgs/${destination}")
    endif()
    if(NOT EXISTS "${destination}")
      file(MAKE_DIRECTORY "${destination}")
    endif()

    if(EXISTS "${file}")
      # determine link name for file including destination path
      get_filename_component(filename "${file}" NAME)
      set(symlink "${destination}/${filename}")
      _ament_cmake_symlink_install_create_symlink("${file}" "${symlink}")
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_targets() can't find '${file}'")
      endif()
    endif()
  endforeach()
endfunction()

function(_ament_cmake_symlink_install_create_symlink absolute_file symlink)
  # register symlink for being removed during install step
  file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/symlink_install_manifest.txt"
    "${symlink}\n")

  # avoid any work if correct symlink is already in place
  if(EXISTS "${symlink}" AND IS_SYMLINK "${symlink}")
    get_filename_component(destination "${symlink}" REALPATH)
    get_filename_component(real_absolute_file "${absolute_file}" REALPATH)
    if(destination STREQUAL real_absolute_file)
      message(STATUS "Up-to-date symlink: ${symlink}")
      return()
    endif()
  endif()

  message(STATUS "Symlinking: ${symlink}")
  if(EXISTS "${symlink}" OR IS_SYMLINK "${symlink}")
    file(REMOVE "${symlink}")
  endif()

  execute_process(
    COMMAND "/usr/bin/cmake" "-E" "create_symlink"
      "${absolute_file}"
      "${symlink}"
  )
  # the CMake command does not provide a return code so check manually
  if(NOT EXISTS "${symlink}" OR NOT IS_SYMLINK "${symlink}")
    get_filename_component(destination "${symlink}" REALPATH)
    message(FATAL_ERROR
      "Could not create symlink '${symlink}' pointing to '${absolute_file}'")
  endif()
endfunction()

# end of template

message(STATUS "Execute custom install script")

# begin of custom install code

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/ublox_ubx_msgs" "DESTINATION" "share/ament_index/resource_index/rosidl_interfaces")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/ublox_ubx_msgs" "DESTINATION" "share/ament_index/resource_index/rosidl_interfaces")

# install(DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_generator_c/ublox_ubx_msgs/" "DESTINATION" "include/ublox_ubx_msgs/ublox_ubx_msgs" "PATTERN" "*.h")
ament_cmake_symlink_install_directory("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_generator_c/ublox_ubx_msgs/" "DESTINATION" "include/ublox_ubx_msgs/ublox_ubx_msgs" "PATTERN" "*.h")

# install(FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh" "DESTINATION" "share/ublox_ubx_msgs/environment")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh" "DESTINATION" "share/ublox_ubx_msgs/environment")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/library_path.dsv" "DESTINATION" "share/ublox_ubx_msgs/environment")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/library_path.dsv" "DESTINATION" "share/ublox_ubx_msgs/environment")

# install(DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_typesupport_fastrtps_c/ublox_ubx_msgs/" "DESTINATION" "include/ublox_ubx_msgs/ublox_ubx_msgs" "PATTERN_EXCLUDE" "*.cpp")
ament_cmake_symlink_install_directory("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_typesupport_fastrtps_c/ublox_ubx_msgs/" "DESTINATION" "include/ublox_ubx_msgs/ublox_ubx_msgs" "PATTERN_EXCLUDE" "*.cpp")

# install(DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_generator_cpp/ublox_ubx_msgs/" "DESTINATION" "include/ublox_ubx_msgs/ublox_ubx_msgs" "PATTERN" "*.hpp")
ament_cmake_symlink_install_directory("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_generator_cpp/ublox_ubx_msgs/" "DESTINATION" "include/ublox_ubx_msgs/ublox_ubx_msgs" "PATTERN" "*.hpp")

# install(DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_typesupport_fastrtps_cpp/ublox_ubx_msgs/" "DESTINATION" "include/ublox_ubx_msgs/ublox_ubx_msgs" "PATTERN_EXCLUDE" "*.cpp")
ament_cmake_symlink_install_directory("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_typesupport_fastrtps_cpp/ublox_ubx_msgs/" "DESTINATION" "include/ublox_ubx_msgs/ublox_ubx_msgs" "PATTERN_EXCLUDE" "*.cpp")

# install(DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_typesupport_introspection_c/ublox_ubx_msgs/" "DESTINATION" "include/ublox_ubx_msgs/ublox_ubx_msgs" "PATTERN" "*.h")
ament_cmake_symlink_install_directory("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_typesupport_introspection_c/ublox_ubx_msgs/" "DESTINATION" "include/ublox_ubx_msgs/ublox_ubx_msgs" "PATTERN" "*.h")

# install(DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_typesupport_introspection_cpp/ublox_ubx_msgs/" "DESTINATION" "include/ublox_ubx_msgs/ublox_ubx_msgs" "PATTERN" "*.hpp")
ament_cmake_symlink_install_directory("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_typesupport_introspection_cpp/ublox_ubx_msgs/" "DESTINATION" "include/ublox_ubx_msgs/ublox_ubx_msgs" "PATTERN" "*.hpp")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/pythonpath.sh" "DESTINATION" "share/ublox_ubx_msgs/environment")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/pythonpath.sh" "DESTINATION" "share/ublox_ubx_msgs/environment")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/pythonpath.dsv" "DESTINATION" "share/ublox_ubx_msgs/environment")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/pythonpath.dsv" "DESTINATION" "share/ublox_ubx_msgs/environment")

# install(DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_python/ublox_ubx_msgs/ublox_ubx_msgs.egg-info/" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_ubx_msgs-0.7.6-py3.10.egg-info")
ament_cmake_symlink_install_directory("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_python/ublox_ubx_msgs/ublox_ubx_msgs.egg-info/" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_ubx_msgs-0.7.6-py3.10.egg-info")

# install(DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_generator_py/ublox_ubx_msgs/" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_ubx_msgs" "PATTERN_EXCLUDE" "*.pyc" "PATTERN_EXCLUDE" "__pycache__")
ament_cmake_symlink_install_directory("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_generator_py/ublox_ubx_msgs/" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_ubx_msgs" "PATTERN_EXCLUDE" "*.pyc" "PATTERN_EXCLUDE" "__pycache__")

# install("TARGETS" "ublox_ubx_msgs__rosidl_typesupport_fastrtps_c__pyext" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_ubx_msgs")
include("/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_symlink_install_targets_0_${CMAKE_INSTALL_CONFIG_NAME}.cmake")

# install("TARGETS" "ublox_ubx_msgs__rosidl_typesupport_introspection_c__pyext" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_ubx_msgs")
include("/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_symlink_install_targets_1_${CMAKE_INSTALL_CONFIG_NAME}.cmake")

# install("TARGETS" "ublox_ubx_msgs__rosidl_typesupport_c__pyext" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_ubx_msgs")
include("/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_symlink_install_targets_2_${CMAKE_INSTALL_CONFIG_NAME}.cmake")

# install(DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_generator_rs/ublox_ubx_msgs/rust" "DESTINATION" "share/ublox_ubx_msgs")
ament_cmake_symlink_install_directory("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" DIRECTORY "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_generator_rs/ublox_ubx_msgs/rust" "DESTINATION" "share/ublox_ubx_msgs")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/CarrSoln.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/CarrSoln.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/CommsPortInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/CommsPortInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/CorStatusInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/CorStatusInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/ESFMeasDataItem.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/ESFMeasDataItem.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/ESFSensorStatus.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/ESFSensorStatus.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/GpsFix.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/GpsFix.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/JamStateCentFreq.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/JamStateCentFreq.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/MapMatching.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/MapMatching.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/MeasxData.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/MeasxData.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/OrbAlmInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/OrbAlmInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/OrbEphInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/OrbEphInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/OrbSVFlag.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/OrbSVFlag.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/OrbSVInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/OrbSVInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/OtherOrbInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/OtherOrbInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/PSMPVT.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/PSMPVT.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/PSMStatus.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/PSMStatus.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/RawxData.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/RawxData.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/RecStat.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/RecStat.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SatFlags.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SatFlags.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SatInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SatInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SBASService.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SBASService.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SBASStatusFlags.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SBASStatusFlags.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SBASSvData.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SBASSvData.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SigData.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SigData.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SigFlags.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SigFlags.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SigLogEvent.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SigLogEvent.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SpartnKeyInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SpartnKeyInfo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SpoofDet.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/SpoofDet.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/TrkStat.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/TrkStat.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXEsfMeas.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXEsfMeas.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXEsfStatus.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXEsfStatus.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXMonComms.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXMonComms.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavEOE.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavEOE.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavClock.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavClock.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavCov.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavCov.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavDOP.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavDOP.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavHPPosECEF.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavHPPosECEF.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavHPPosLLH.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavHPPosLLH.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavOdo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavOdo.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavOrb.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavOrb.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavPosECEF.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavPosECEF.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavPosLLH.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavPosLLH.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavPVT.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavPVT.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavRelPosNED.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavRelPosNED.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavSat.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavSat.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavSBAS.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavSBAS.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavSig.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavSig.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavStatus.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavStatus.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavSvin.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavSvin.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavTimeUTC.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavTimeUTC.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavVelECEF.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavVelECEF.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavVelNED.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXNavVelNED.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmCor.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmCor.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmMeasx.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmMeasx.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmPmp.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmPmp.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmQzssl6.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmQzssl6.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmRawx.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmRawx.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmRTCM.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmRTCM.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmSfrbx.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmSfrbx.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmSpartn.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmSpartn.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmSpartnKey.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXRxmSpartnKey.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXSecSig.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXSecSig.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXSecSigLog.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXSecSigLog.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXSecUniqid.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UBXSecUniqid.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UtcStd.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_adapter/ublox_ubx_msgs/msg/UtcStd.idl" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/CarrSoln.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/CarrSoln.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/CommsPortInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/CommsPortInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/CorStatusInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/CorStatusInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/ESFMeasDataItem.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/ESFMeasDataItem.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/ESFSensorStatus.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/ESFSensorStatus.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/GpsFix.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/GpsFix.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/JamStateCentFreq.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/JamStateCentFreq.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/MapMatching.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/MapMatching.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/MeasxData.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/MeasxData.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/OrbAlmInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/OrbAlmInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/OrbEphInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/OrbEphInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/OrbSVFlag.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/OrbSVFlag.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/OrbSVInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/OrbSVInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/OtherOrbInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/OtherOrbInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/PSMPVT.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/PSMPVT.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/PSMStatus.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/PSMStatus.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/RawxData.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/RawxData.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/RecStat.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/RecStat.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SatFlags.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SatFlags.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SatInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SatInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SBASService.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SBASService.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SBASStatusFlags.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SBASStatusFlags.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SBASSvData.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SBASSvData.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SigData.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SigData.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SigFlags.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SigFlags.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SigLogEvent.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SigLogEvent.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SpartnKeyInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SpartnKeyInfo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SpoofDet.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/SpoofDet.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/TrkStat.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/TrkStat.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXEsfMeas.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXEsfMeas.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXEsfStatus.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXEsfStatus.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXMonComms.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXMonComms.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavEOE.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavEOE.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavClock.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavClock.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavCov.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavCov.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavDOP.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavDOP.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavHPPosECEF.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavHPPosECEF.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavHPPosLLH.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavHPPosLLH.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavOdo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavOdo.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavOrb.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavOrb.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavPosECEF.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavPosECEF.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavPosLLH.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavPosLLH.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavPVT.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavPVT.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavRelPosNED.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavRelPosNED.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavSat.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavSat.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavSBAS.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavSBAS.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavSig.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavSig.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavStatus.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavStatus.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavSvin.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavSvin.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavTimeUTC.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavTimeUTC.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavVelECEF.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavVelECEF.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavVelNED.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXNavVelNED.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmCor.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmCor.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmMeasx.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmMeasx.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmPmp.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmPmp.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmQzssl6.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmQzssl6.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmRawx.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmRawx.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmRTCM.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmRTCM.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmSfrbx.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmSfrbx.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmSpartn.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmSpartn.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmSpartnKey.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXRxmSpartnKey.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXSecSig.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXSecSig.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXSecSigLog.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXSecSigLog.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXSecUniqid.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UBXSecUniqid.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UtcStd.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/msg/UtcStd.msg" "DESTINATION" "share/ublox_ubx_msgs/msg")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ublox_ubx_msgs" "DESTINATION" "share/ament_index/resource_index/package_run_dependencies")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ublox_ubx_msgs" "DESTINATION" "share/ament_index/resource_index/package_run_dependencies")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ublox_ubx_msgs" "DESTINATION" "share/ament_index/resource_index/parent_prefix_path")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ublox_ubx_msgs" "DESTINATION" "share/ament_index/resource_index/parent_prefix_path")

# install(FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh" "DESTINATION" "share/ublox_ubx_msgs/environment")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh" "DESTINATION" "share/ublox_ubx_msgs/environment")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/ament_prefix_path.dsv" "DESTINATION" "share/ublox_ubx_msgs/environment")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/ament_prefix_path.dsv" "DESTINATION" "share/ublox_ubx_msgs/environment")

# install(FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh" "DESTINATION" "share/ublox_ubx_msgs/environment")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh" "DESTINATION" "share/ublox_ubx_msgs/environment")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/path.dsv" "DESTINATION" "share/ublox_ubx_msgs/environment")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/path.dsv" "DESTINATION" "share/ublox_ubx_msgs/environment")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/local_setup.bash" "DESTINATION" "share/ublox_ubx_msgs")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/local_setup.bash" "DESTINATION" "share/ublox_ubx_msgs")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/local_setup.sh" "DESTINATION" "share/ublox_ubx_msgs")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/local_setup.sh" "DESTINATION" "share/ublox_ubx_msgs")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/local_setup.zsh" "DESTINATION" "share/ublox_ubx_msgs")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/local_setup.zsh" "DESTINATION" "share/ublox_ubx_msgs")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/local_setup.dsv" "DESTINATION" "share/ublox_ubx_msgs")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/local_setup.dsv" "DESTINATION" "share/ublox_ubx_msgs")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/package.dsv" "DESTINATION" "share/ublox_ubx_msgs")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_environment_hooks/package.dsv" "DESTINATION" "share/ublox_ubx_msgs")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_index/share/ament_index/resource_index/packages/ublox_ubx_msgs" "DESTINATION" "share/ament_index/resource_index/packages")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_index/share/ament_index/resource_index/packages/ublox_ubx_msgs" "DESTINATION" "share/ament_index/resource_index/packages")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_cmake/rosidl_cmake-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_cmake/rosidl_cmake-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_cmake/rosidl_cmake_aggregate_target-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/rosidl_cmake/rosidl_cmake_aggregate_target-extras.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")

# install(FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_core/ublox_ubx_msgsConfig.cmake" "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_core/ublox_ubx_msgsConfig-version.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_core/ublox_ubx_msgsConfig.cmake" "/home/clara/AutonomousPathPlanning/src/build/ublox_ubx_msgs/ament_cmake_core/ublox_ubx_msgsConfig-version.cmake" "DESTINATION" "share/ublox_ubx_msgs/cmake")

# install(FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/package.xml" "DESTINATION" "share/ublox_ubx_msgs")
ament_cmake_symlink_install_files("/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs" FILES "/home/clara/AutonomousPathPlanning/src/ublox_dgnss/ublox_ubx_msgs/package.xml" "DESTINATION" "share/ublox_ubx_msgs")
