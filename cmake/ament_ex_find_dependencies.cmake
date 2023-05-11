#
# Invoke find_package() for all build and buildtool dependencies.
#
# All found package names are appended to the
# ``${PROJECT_NAME}_FOUND_BUILD_DEPENDS`` /
# ``${PROJECT_NAME}_FOUND_BUILDTOOL_DEPENDS`` /
# ``${PROJECT_NAME}_FOUND_TEST_DEPENDS`` variables.
#
# The content of the package specific variables of build dependencies
# ending with ``_TARGETS`` if dependency export targets, or with ``_DEFINITIONS``, 
# ``_INCLUDE_DIRS`` and ``_LIBRARIES`` otherwise, are appended to the same variables 
# starting with ``${PROJECT_NAME}_FOUND_``.
#
# @public
#
macro(ament_ex_find_dependencies)
  set(_ARGN "${ARGN}")
  if(_ARGN)
    message(FATAL_ERROR "ament_ex_find_dependencies() called with unused arguments: ${_ARGN}")
  endif()

  if(NOT _AMENT_PACKAGE_NAME)
    ament_package_xml()
  endif()

  # try to find_package() all build dependencies
  foreach(_dep ${${PROJECT_NAME}_BUILD_DEPENDS})
    set(_REQUIRED_KEYWORD "")
    if(_dep IN_LIST _ARG_REQUIRED)
      set(_REQUIRED_KEYWORD "REQUIRED")
    endif()
    find_package(${_dep} QUIET ${_REQUIRED_KEYWORD})
    if(${_dep}_FOUND)
      list(APPEND ${PROJECT_NAME}_FOUND_BUILD_DEPENDS ${_dep})
      # if a package provides modern CMake interface targets use them
      # exclusively assuming the classic CMake variables only exist for
      # backward compatibility
      if(NOT "${${_dep}_TARGETS}" STREQUAL "")
        list(APPEND ${PROJECT_NAME}_FOUND_TARGETS ${${_dep}_TARGETS})
      else()
        list(APPEND ${PROJECT_NAME}_FOUND_DEFINITIONS ${${_dep}_DEFINITIONS})
        list(APPEND ${PROJECT_NAME}_FOUND_INCLUDE_DIRS ${${_dep}_INCLUDE_DIRS})
        list(APPEND ${PROJECT_NAME}_FOUND_LIBRARIES ${${_dep}_LIBRARIES})
      endif()
    endif()
  endforeach()

  # try to find_package() all buildtool dependencies
  foreach(_dep ${${PROJECT_NAME}_BUILDTOOL_DEPENDS})
    set(_REQUIRED_KEYWORD "")
    if(_dep IN_LIST _ARG_REQUIRED)
      set(_REQUIRED_KEYWORD "REQUIRED")
    endif()
    find_package(${_dep} QUIET ${_REQUIRED_KEYWORD})
    if(${_dep}_FOUND)
      list(APPEND ${PROJECT_NAME}_FOUND_BUILDTOOL_DEPENDS ${_dep})
    endif()
  endforeach()

  # try to find_package() all test dependencies
  foreach(_dep ${${PROJECT_NAME}_TEST_DEPENDS})
    find_package(${_dep} QUIET)
    if(${_dep}_FOUND)
      list(APPEND ${PROJECT_NAME}_FOUND_TEST_DEPENDS ${_dep})
    endif()
  endforeach()

endmacro()
