# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rovc_comm_mqtt_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rovc_comm_mqtt_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rovc_comm_mqtt_FOUND FALSE)
  elseif(NOT rovc_comm_mqtt_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rovc_comm_mqtt_FOUND FALSE)
  endif()
  return()
endif()
set(_rovc_comm_mqtt_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rovc_comm_mqtt_FIND_QUIETLY)
  message(STATUS "Found rovc_comm_mqtt: 0.0.0 (${rovc_comm_mqtt_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rovc_comm_mqtt' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rovc_comm_mqtt_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rovc_comm_mqtt_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rovc_comm_mqtt_DIR}/${_extra}")
endforeach()
