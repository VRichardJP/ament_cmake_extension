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
  # extract dependencies
  cmake_parse_arguments(_ARG "INTERFACE;PUBLIC;SYSTEM" "" "" ${ARGN})
  set(_deps ${_ARG_UNPARSED_ARGUMENTS})

  # if target is a library the dependencies are most likely required downstream
  get_target_property(target_type ${target} TYPE)
  if (NOT "${target_type}" STREQUAL "EXECUTABLE")
    ament_export_dependencies(${deps})
  endif()

  # Let ament_target_dependencies figure out what is the correct way to include
  # the dependency. Its implementation is quite complex, but basically what it
  # does is:
  # A. If the dependency defines modern CMake targets, then simply do:
  #      target_link_libraries(${target} ${dep}_TARGETS)
  # B. Otherwise use the classic CMake variables:
  #      target_compile_definitions(${target} ${${dep}_DEFINITIONS})
  #      target_include_directories(${target} ${${dep}_INCLUDE_DIRS})
  #      target_link_libraries(${target} ${${dep}_LIBRARIES})
  ament_target_dependencies(${target} ${ARGN})

endmacro()
