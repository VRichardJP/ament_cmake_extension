#
# Convenience wrapper over add_library(). Accept the same parameters.
# Automatically include, link package dependencies and install.
#
# @public
#
macro(ament_ex_add_library target)

  add_library(${target} ${ARGN})

  # INTERFACE libraries must use INTERFACE keyword
  get_target_property(_type ${target} TYPE)
  if("${_type}" STREQUAL "INTERFACE_LIBRARY")
    set(_keyword INTERFACE)
  else()
    set(_keyword PUBLIC)
  endif()
  
  # add "include" directory
  target_include_directories(${target} ${_keyword} 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  # include/link all package dependencies
  ament_ex_target_add_package_dependencies(${target})

  ament_ex_install_target(${target})

endmacro()
