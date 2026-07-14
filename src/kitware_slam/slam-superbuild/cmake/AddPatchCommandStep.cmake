function (reset_patches_step _lib_name)
  find_package(Git QUIET)
  if (NOT GIT_FOUND)
    mark_as_advanced(CLEAR GIT_EXECUTABLE)
    message(FATAL_ERROR "Could not find git executable.  Please set GIT_EXECUTABLE.")
  endif ()

  ExternalProject_Add_Step("${_lib_name}" "${_lib_name}-reset-patches"
    COMMAND "${GIT_EXECUTABLE}" reset --hard
    DEPENDEES patch
    DEPENDERS configure
    COMMENT "Reset all patches for ${_lib_name}"
    WORKING_DIRECTORY <SOURCE_DIR>)
endfunction()

function (add_patch_command_step _lib_name _patch_name)
  find_package(Git QUIET)
  if (NOT GIT_FOUND)
    mark_as_advanced(CLEAR GIT_EXECUTABLE)
    message(FATAL_ERROR "Could not find git executable.  Please set GIT_EXECUTABLE.")
  endif ()

  string(TOLOWER "${_lib_name}" patch_prefix)
  ExternalProject_Add_Step("${_lib_name}" "${patch_prefix}-patch-${_patch_name}"
    COMMAND "${GIT_EXECUTABLE}"
      --git-dir= # Make `git` think it is not in a repository.
      apply
      # Necessary for applying patches to windows-newline files.
      --whitespace=fix
      -p1
      "${CMAKE_CURRENT_LIST_DIR}/patches/${patch_prefix}-${_patch_name}.patch"
    DEPENDEES patch "${_lib_name}-reset-patches"
    DEPENDERS configure
    COMMENT "Apply ${_patch_name} for ${_lib_name}"
    WORKING_DIRECTORY <SOURCE_DIR>)
endfunction()
