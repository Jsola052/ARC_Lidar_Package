function (find_dependencies name)

  # Grap arguments
  set(grab)
  set(depends)
  foreach (arg IN LISTS ARGN)
    if (arg STREQUAL "DEPENDS")
      set(grab depends)
    elseif(grab)
      list(APPEND "${grab}" "${arg}")
    endif()
  endforeach()

  # Seach for INSTALL_dep
  set(depends_on)
  foreach(dep IN LISTS depends)
    if (DEFINED "INSTALL_${dep}" AND "${INSTALL_${dep}}")
      list(APPEND depends_on ${dep})
    endif()
  endforeach()

  set("${name}_DEPENDS" ${depends_on} PARENT_SCOPE)

endfunction()
