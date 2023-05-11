#
# Install a target
#
# :param target: the name of the target
# :type target: string
# :param NO_EXPORT: if set, the target will not be exported to downstream packages (e.g. executable)
# :type NO_EXPORT: option
#
# @public
#
macro(ament_ex_install_target target)
  cmake_parse_arguments(_ARG "NO_EXPORT" "" "" ${ARGN})
  if(_ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_ex_install_target() called with "
      "unused arguments: ${_ARG_UNPARSED_ARGUMENTS}")
  endif()

  if (_ARG_NO_EXPORT)
    install(
      TARGETS ${target}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
  else()
    install(
      TARGETS ${target}
      EXPORT ${PROJECT_NAME}Targets
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
    # will be exported to downstream packages
    list(APPEND ${PROJECT_NAME}_TARGETS "${target}")
  endif()


endmacro()
