set(lidarslam_sources)

set(lidarslam_classes
  vtkKeypointMapBuilder
  vtkSlam
  vtkSlamManager
  vtkMiniSlam
  vtkSpinningSensorKeypointExtractor
)

set(lidarslam_headers)
set(lidarslam_templates)

set(lidarslam_xml
  Slam.xml
  MiniSlam.xml
)

option(ENABLE_KEYPOINT_MAP_BUILDER_FILTER "This filter can be used to debug SLAM keypoints" OFF)
if (ENABLE_KEYPOINT_MAP_BUILDER_FILTER)
  list(APPEND lidarslam_xml
    KeypointMapBuilder.xml
  )
endif ()

# Create Module
vtk_module_add_module(vtkLidarSlam
  FORCE_STATIC
  SOURCES ${lidarslam_sources} # Will not trigger a PV wrapping
  CLASSES ${lidarslam_classes} # Those will be PV ClientServer wrapped #shorcut for sources x.cxx x.h
  HEADERS ${lidarslam_headers} # Those will be PV ClientServer wrapped
)
paraview_add_server_manager_xmls(
  MODULE vtkLidarSlam
  XMLS  ${lidarslam_xml}
)

target_link_libraries(vtkLidarSlam PUBLIC LidarSlam)
