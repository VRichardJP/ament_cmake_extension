#
# Convenience wrapper over add_executable(). Accept the same parameters.
# Automatically include, link package dependencies and install.
#
# @public
#
macro(ament_ex_add_executable target)

  add_executable(${target} ${ARGN})

  # add "include" directory
  target_include_directories(${target} PRIVATE include)

  # include/link all package dependencies
  ament_ex_target_add_package_dependencies(${target})
  
  ament_ex_install_target(${target} NO_EXPORT)

endmacro()
