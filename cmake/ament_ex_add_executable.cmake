#
# Convenience wrapper over ``add_executable()``. Accept the same parameters.
# Automatically include, link package dependencies and install.
#
# @public
#
macro(ament_ex_add_executable target)

  add_executable(${target} ${ARGN})

  # add "./include" directory, if any
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_include_directories(${target} PRIVATE include)
  endif()

  # include/link all package dependencies
  ament_ex_target_add_package_dependencies(${target} BUILD_DEPS)
  
  # install the target but don't export it downstream
  ament_ex_install_targets(NO_EXPORT ${target})

endmacro()
