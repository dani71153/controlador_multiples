# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_controlador_multiples_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED controlador_multiples_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(controlador_multiples_FOUND FALSE)
  elseif(NOT controlador_multiples_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(controlador_multiples_FOUND FALSE)
  endif()
  return()
endif()
set(_controlador_multiples_CONFIG_INCLUDED TRUE)

# output package information
if(NOT controlador_multiples_FIND_QUIETLY)
  message(STATUS "Found controlador_multiples: 0.0.0 (${controlador_multiples_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'controlador_multiples' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT controlador_multiples_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(controlador_multiples_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${controlador_multiples_DIR}/${_extra}")
endforeach()
