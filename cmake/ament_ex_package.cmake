#
# Export package dependencies and targets and invoke ``ament_package()``.
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
  cmake_parse_arguments(_ARG "" "" "INSTALL_TO_SHARE" ${ARGN})
  # NOTE: ament_export_targets() override _ARG_UNPARSED_ARGUMENTS, so we need to save it here
  set(_ament_package_args ${_ARG_UNPARSED_ARGUMENTS})

  # install directories to share
  foreach(_dir ${_ARG_INSTALL_TO_SHARE})
    install(
      DIRECTORY "${_dir}"
      DESTINATION "share/${PROJECT_NAME}"
    )
  endforeach()

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
  endif()

  ament_package(${_ament_package_args})
endmacro()