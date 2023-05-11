#
# Add all package dependencies to the target: definitions, includes, libraries, targets.
# Optional flags:
# - TEST: also add test dependencies
#
# :param target: the name of the target
# :type target: string
# :param TEST: if set, also add test dependencies
# :type TEST: option
#
# @public
#
macro(ament_ex_target_add_package_dependencies target)
  cmake_parse_arguments(_ARG "TEST" "" "" ${ARGN})
  if(_ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_ex_target_add_package_dependencies() called with "
      "unused arguments: ${_ARG_UNPARSED_ARGUMENTS}")
  endif()

  # INTERFACE libraries need the INTERFACE keyword
  get_target_property(_type ${target} TYPE)
  if("${_type}" STREQUAL "INTERFACE_LIBRARY")
    set(_keyword INTERFACE)
  else()
    unset(_keyword)
  endif()

  # modern cmake, pull automatically all necessary dependencies (include, link...)
  if(NOT "${${PROJECT_NAME}_FOUND_TARGETS}" STREQUAL "")
    target_link_libraries(${target} ${_keyword} ${${PROJECT_NAME}_FOUND_TARGETS})
  endif()

  # let ament_target_dependencies figure out what is the correct include/link required for each build dependency
  if(NOT "${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}" STREQUAL "")
    ament_target_dependencies(${target} SYSTEM ${_keyword} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endif()

  # same for tests
  if(_ARG_TEST AND NOT "${${PROJECT_NAME}_FOUND_TEST_DEPENDS}" STREQUAL "")
    ament_target_dependencies(${target} SYSTEM ${_keyword} ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
  endif()

endmacro()
