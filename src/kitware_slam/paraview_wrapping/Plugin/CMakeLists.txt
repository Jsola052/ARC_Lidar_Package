if (PARAVIEW_USE_QT)
  set(interfaces)
  set(sources)
  set(ui_files
    UserInterface/Resources/UI/sqAboutSlamDialog.ui
    UserInterface/Resources/UI/sqPresetDialog.ui
    UserInterface/Resources/UI/sqSlamToolbar.ui
    UserInterface/Resources/UI/sqSeedHandlePropertyWidget.ui
  )

  set(subdir_sources
    sqAboutSlamAction
    sqAboutSlamDialog
    sqAddSlamReaction
    sqAppendAnchorPropertyWidget
    sqButtonPausePropertyWidget
    sqInitializeDialogReaction
    sqPickTrajectoryIdPropertyWidget
    sqPresetDialog
    sqPresetDialogReaction
    sqSeedHandlePropertyWidget
    sqSelectTablePortDialog
    sqSlamManager
    sqSlamStarter
    sqSlamToolbar
    sqSubsetPropertiesDialog
    sqTablePipelineProxyModel
    sqTriggerSlamCommandReaction
    sqOptionalInputDecorator
  )

  set(extensions cxx h)
  foreach (filename ${subdir_sources})
    foreach (ext IN LISTS extensions)
      list(APPEND sources "${CMAKE_CURRENT_SOURCE_DIR}/UserInterface/${filename}.${ext}")
    endforeach ()
  endforeach ()

  paraview_plugin_add_property_widget(
    KIND WIDGET
    TYPE "command_pause_button"
    CLASS_NAME sqButtonPausePropertyWidget
    INTERFACES pause_widget_interfaces
    SOURCES  pause_widget_sources)

  paraview_plugin_add_property_widget(
    KIND WIDGET
    TYPE "slam_seed_handle"
    CLASS_NAME sqSeedHandlePropertyWidget
    INTERFACES seed_handle_widget_interfaces
    SOURCES seed_handle_widget_sources)

  paraview_plugin_add_property_widget(
    KIND WIDGET_DECORATOR
    TYPE "OptionalInputDecorator"
    CLASS_NAME sqOptionalInputDecorator
    INTERFACES optional_input_decorator_interfaces
    SOURCES optional_input_decorator_sources)

  paraview_plugin_add_property_widget(
    KIND WIDGET
    TYPE "slam_pick_trajectory_id_widget"
    CLASS_NAME sqPickTrajectoryIdPropertyWidget
    INTERFACES pick_value_prop_widget_interfaces
    SOURCES pick_value_prop_widget_sources)

  paraview_plugin_add_property_widget(
    KIND WIDGET
    TYPE "slam_append_anchor_widget"
    CLASS_NAME sqAppendAnchorPropertyWidget
    INTERFACES append_anchor_prop_widget_interfaces
    SOURCES append_anchor_prop_widget_sources)


  paraview_plugin_add_action_group(
    CLASS_NAME sqAboutSlamAction
    GROUP_NAME "MenuBar/Help"
    INTERFACES about_action_interfaces
    SOURCES about_action_sources
  )

  paraview_plugin_add_auto_start(
    CLASS_NAME sqSlamStarter
    STARTUP onStartup
    SHUTDOWN onShutdown
    INTERFACES autostart_interface
    SOURCES autostart_sources)

  paraview_plugin_add_toolbar(
    CLASS_NAME sqSlamToolbar
    INTERFACES toolbar_interfaces
    SOURCES toolbar_sources)

  list(APPEND interfaces
    ${autostart_interface}
    ${pause_widget_interfaces}
    ${seed_handle_widget_interfaces}
    ${optional_input_decorator_interfaces}
    ${pick_value_prop_widget_interfaces}
    ${append_anchor_prop_widget_interfaces}
    ${about_action_interfaces}
    ${toolbar_interfaces}
  )
  list(APPEND sources
    ${autostart_sources}
    ${pause_widget_sources}
    ${seed_handle_widget_sources}
    ${optional_input_decorator_sources}
    ${pick_value_prop_widget_sources}
    ${append_anchor_prop_widget_sources}
    ${about_action_sources}
    ${toolbar_sources}
  )

  set(qrc_files
    UserInterface/Resources/sqResources.qrc
  )

  list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS
    "${CMAKE_CURRENT_SOURCE_DIR}/UserInterface/Resources/UI")
endif ()

# Create Plugin
set(CMAKE_PROJECT_VERSION ${LidarSlam_VERSION})
paraview_add_plugin(LidarSlamPlugin
  REQUIRED_ON_CLIENT
  REQUIRED_ON_SERVER
  MODULE_INSTALL_EXPORT LidarSlamLib
  VERSION ${CMAKE_PROJECT_VERSION}
  UI_RESOURCES ${qrc_files}
  UI_FILES ${ui_files}
  UI_INTERFACES ${interfaces}
  SOURCES ${sources}
  MODULES
    vtkLidarSlam
    vtkSlamUtilities
  MODULE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/vtkLidarSlam/vtk.module"
    "${CMAKE_CURRENT_SOURCE_DIR}/vtkSlamUtilities/vtk.module"
  DOCUMENTATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/doc/"
)

if (PARAVIEW_USE_QT)
  # Libraries required by UserInterface code
  set(required_modules
    ParaView::pqApplicationComponents
    ParaView::pqComponents
    ParaView::pqCore
    ParaView::RemotingServerManager
    ParaView::RemotingViews
    ParaView::RemotingMisc
    VTK::RenderingCore
  )

  target_link_libraries(LidarSlamPlugin PRIVATE ${required_modules})
  target_include_directories(LidarSlamPlugin PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/UserInterface")
endif ()
