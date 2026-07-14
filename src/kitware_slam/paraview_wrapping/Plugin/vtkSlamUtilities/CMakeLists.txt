set(headers)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/vtkSlamInfo.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/vtkSlamInfo.h"
  @ONLY)
list(APPEND headers
  "${CMAKE_CURRENT_BINARY_DIR}/vtkSlamInfo.h")


set(classes
  vtkLoopClosureActors
  vtkSlamFinder
)

vtk_module_add_module(vtkSlamUtilities
  FORCE_STATIC
  HEADERS ${headers}
  CLASSES ${classes}
)
