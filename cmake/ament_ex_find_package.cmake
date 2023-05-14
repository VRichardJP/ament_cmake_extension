#
# Convenience wrapper over ``find_package()``. Accept the same parameters.
# Export the package for downstream packages.
#
# :param ARGN: parameters to ``find_package()``
# :type ARGN: list of strings
#
# @public
#
macro(ament_ex_find_package package)
  find_package(${package} ${ARGN})
  ament_export_dependencies(${package})
endmacro()
