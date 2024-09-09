# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Kinematic_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Kinematic_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Kinematic_FOUND FALSE)
  elseif(NOT Kinematic_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Kinematic_FOUND FALSE)
  endif()
  return()
endif()
set(_Kinematic_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Kinematic_FIND_QUIETLY)
  message(STATUS "Found Kinematic: 0.0.0 (${Kinematic_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Kinematic' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT Kinematic_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Kinematic_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Kinematic_DIR}/${_extra}")
endforeach()
