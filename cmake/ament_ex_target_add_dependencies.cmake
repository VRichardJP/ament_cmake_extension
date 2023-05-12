#
# Correctly add dependencies definitions, includes, libraries to the target.
# Accept the same parameters than ament_target_dependencies.
# If the target is a library, the dependencies will be exported for downstream packages.
#
# WARNING: despite its name, this function is very different from CMake builtin 
# add_dependencies() function
#
# :param target: the target name
# :type target: string
# :param ARGN: parameters to ament_target_dependencies
# :type ARGN: list of strings
#
# @public
#
macro(ament_ex_target_add_dependencies target)
  # passed down to ament_target_dependencies()
  set(_ament_target_dependencies_args ${ARGN})

  # extract dependencies
  cmake_parse_arguments(_ARG "INTERFACE;PUBLIC;SYSTEM" "" "" ${ARGN})
  set(_deps ${_ARG_UNPARSED_ARGUMENTS})

  # if target is a library the dependencies are most likely required downstream
  get_target_property(target_type ${target} TYPE)
  if (NOT "${target_type}" STREQUAL "EXECUTABLE")
    ament_export_dependencies(${deps})
  endif()

  ament_target_dependencies(${target} ${ARGN})

endmacro()
