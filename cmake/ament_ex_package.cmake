#
# Export dependencies and targets and invoke ``ament_package()``.
#
# :param INSTALL_TO_SHARE: a list of directories to be installed to the
#   package's share directory
# :type INSTALL_TO_SHARE: list of strings
# :param ARGN: any other arguments are passed through to ament_package()
# :type ARGN: list of strings
#
# @public
#
macro(ament_ex_package)
  # NOTE: Can't use prefix _ARG because ament_export_targets() below would overwrite 
  # the variable _ARG_UNPARSED_ARGUMENTS before it is passed down to ament_package()
  # This would not be a problem if we used functions instead of macros. Unfortunately,
  # the whole ament cmake framework only works with macros :(
  cmake_parse_arguments(ARG "" "" "INSTALL_TO_SHARE" ${ARGN})
  # passing all unparsed arguments to ament_package()

  # export all found build dependencies which are also run dependencies
  set(_run_depends
    ${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS}
    ${${PROJECT_NAME}_BUILDTOOL_EXPORT_DEPENDS}
    ${${PROJECT_NAME}_EXEC_DEPENDS})
  foreach(_dep
      ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
      ${${PROJECT_NAME}_FOUND_BUILDTOOL_DEPENDS})
    if(_dep IN_LIST _run_depends)
      ament_export_dependencies("${_dep}")
    endif()
  endforeach()

  if(NOT "${${PROJECT_NAME}_TARGETS}" STREQUAL "")
    # export all targets for downstream packages
    ament_export_targets(${PROJECT_NAME}Targets)

    # don't forget to install "include" directory
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
      install(DIRECTORY include/ DESTINATION include)
    endif()
  endif()

  # install directories to share
  foreach(_dir ${ARG_INSTALL_TO_SHARE})
    install(
      DIRECTORY "${_dir}"
      DESTINATION "share/${PROJECT_NAME}"
    )
  endforeach()

  ament_package(${ARG_UNPARSED_ARGUMENTS})
endmacro()