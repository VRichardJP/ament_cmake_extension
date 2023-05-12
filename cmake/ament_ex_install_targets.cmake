#
# Install and export targets
#
# :param ARGN: targets that will be installed and exported
# :type ARGN: list of strings
# :param NO_EXPORT: targets that will be installed but not be exported to 
#                   downstream packages (e.g. executable)
# :type NO_EXPORT: list of strings
#
# @public
#
macro(ament_ex_install_targets)
  cmake_parse_arguments(_ARG "" "" "NO_EXPORT" ${ARGN})
  set(_not_exported_targets ${_ARG_NO_EXPORT})
  set(_exported_targets ${_ARG_UNPARSED_ARGUMENTS})

  if (_not_exported_targets)
    install(
      TARGETS ${_not_exported_targets}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
  endif()
  
  if(_exported_targets)
    install(
      TARGETS ${_exported_targets}
      EXPORT ${PROJECT_NAME}Targets
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
    # will be exported to downstream packages
    list(APPEND ${PROJECT_NAME}_TARGETS "${_exported_targets}")
  endif()

endmacro()
