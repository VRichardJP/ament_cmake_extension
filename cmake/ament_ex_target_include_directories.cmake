#
# Properly include and install include directory so that downstream packages can
# access them
#
# :param target: the target name
# :type target: string
# :param ARGN: list of directories to include (no keyword allowed)
# :type ARGN: list of strings
#
# @public
#
macro(ament_ex_target_include_directories target)

  # INTERFACE libraries must use INTERFACE keyword
  get_target_property(_type ${target} TYPE)
  if("${_type}" STREQUAL "INTERFACE_LIBRARY")
    set(_keyword INTERFACE)
  else()
    set(_keyword PUBLIC)
  endif()

  foreach(_dir ${ARGN})
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}")
      message(FATAL_ERROR "${CMAKE_CURRENT_SOURCE_DIR}/${_dir} does not exist. "
        "ament_ex_target_include_directories() is only for relative paths. Did "
        "you ment to use target_include_directories()?")
    endif()

    # make sure downstream package access the installed directory
    target_include_directories(${target} ${_keyword} 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${_dir}>
      $<INSTALL_INTERFACE:${_dir}>)

    # do the actual installation 
    install(DIRECTORY ${_dir}/ DESTINATION ${_dir})
  endforeach()
  
endmacro()
