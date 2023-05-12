#
# Correctly add all package dependencies (definitions, includes, libraries) to the target.
# Must provide BUILD_DEPS or/and TEST_DEPS flag
#
# :param target: the name of the target
# :type target: string
# :param BUILD_DEPS: if set, add build dependencies
# :type BUILD_DEPS: option
# :param TEST_DEPS: if set, add test dependencies
# :type TEST_DEPS: option
#
# @public
#
macro(ament_ex_target_add_package_dependencies target)
  cmake_parse_arguments(_ARG "BUILD_DEPS;TEST_DEPS" "" "" ${ARGN})
  if(_ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_ex_target_add_package_dependencies() called with "
      "unused arguments: ${_ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT _ARG_BUILD_DEPS AND NOT _ARG_TEST_DEPS)
    message(FATAL_ERROR "ament_ex_target_add_package_dependencies() called with "
      "no BUILD_DEPS nor TEST_DEPS option")
  endif()

  # INTERFACE libraries need the INTERFACE keyword
  get_target_property(_type ${target} TYPE)
  if("${_type}" STREQUAL "INTERFACE_LIBRARY")
    set(_keyword INTERFACE)
  else()
    unset(_keyword)
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
  if(_ARG_BUILD_DEPS AND NOT "${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}" STREQUAL "")
    ament_target_dependencies(${target} SYSTEM ${_keyword} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endif()

  # Same for tests
  if(_ARG_TEST_DEPS AND NOT "${${PROJECT_NAME}_FOUND_TEST_DEPENDS}" STREQUAL "")
    ament_target_dependencies(${target} SYSTEM ${_keyword} ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
  endif()

endmacro()
