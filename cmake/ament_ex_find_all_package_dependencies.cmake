#
# Invoke ``find_package()`` for all build and buildtool dependencies.
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
macro(ament_ex_find_all_package_dependencies)
  set(_ARGN "${ARGN}")
  if(_ARGN)
    message(FATAL_ERROR "ament_ex_find_all_package_dependencies() called with unused arguments: ${_ARGN}")
  endif()

  if(NOT _AMENT_PACKAGE_NAME)
    ament_package_xml()
  endif()

  # try to find_package() all build dependencies
  foreach(_dep ${${PROJECT_NAME}_BUILD_DEPENDS})
    find_package(${_dep} QUIET)
    if(${_dep}_FOUND)
      list(APPEND ${PROJECT_NAME}_FOUND_BUILD_DEPENDS ${_dep})
    endif()
  endforeach()

  # try to find_package() all buildtool dependencies
  foreach(_dep ${${PROJECT_NAME}_BUILDTOOL_DEPENDS})
    find_package(${_dep} QUIET)
    if(${_dep}_FOUND)
      list(APPEND ${PROJECT_NAME}_FOUND_BUILDTOOL_DEPENDS ${_dep})
    endif()
  endforeach()

  if(BUILD_TESTING)
    # try to find_package() all test dependencies
    foreach(_dep ${${PROJECT_NAME}_TEST_DEPENDS})
      find_package(${_dep} QUIET)
      if(${_dep}_FOUND)
        list(APPEND ${PROJECT_NAME}_FOUND_TEST_DEPENDS ${_dep})
      endif()
    endforeach()
  endif()

endmacro()
