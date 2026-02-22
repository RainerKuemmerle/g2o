# Use CMake automoc/autouic so moc/uic are generated with the configured Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_library(viewer_library ${G2O_LIB_TYPE}
  g2o_viewer_api.h
  g2o_qglviewer.cpp g2o_qglviewer.h
  qglviewer_shim.cpp qglviewer_shim.h
  main_window.cpp main_window.h
  stream_redirect.cpp stream_redirect.h
  gui_hyper_graph_action.cpp gui_hyper_graph_action.h
  abstract_properties_widget.cpp abstract_properties_widget.h
  properties_widget.cpp properties_widget.h
  viewer_properties_widget.cpp viewer_properties_widget.h
  run_g2o_viewer.cpp run_g2o_viewer.h
  ${UI_HEADERS} ${UI_SOURCES}
)
set_target_properties(viewer_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}viewer)
set_target_properties(viewer_library PROPERTIES
  VERSION ${G2O_LIB_VERSION}
  SOVERSION ${G2O_LIB_SOVERSION})
if (APPLE)
  set_target_properties(viewer_library PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

add_executable(g2o_viewer
  g2o_viewer.cpp
)

set_target_properties(g2o_viewer PROPERTIES OUTPUT_NAME g2o_viewer)

find_package(Qt6 COMPONENTS Widgets OpenGLWidgets REQUIRED)
target_link_libraries(viewer_library PUBLIC core g2o_cli_library Qt6::Widgets Qt6::OpenGLWidgets)
target_link_libraries(viewer_library PUBLIC opengl_helper)
target_link_libraries(viewer_library PRIVATE CLI11::CLI11)
target_link_libraries(g2o_viewer viewer_library)

install(TARGETS viewer_library
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)
install(TARGETS g2o_viewer
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/apps/g2o_viewer)
install(FILES ${UI_HEADERS} DESTINATION ${INCLUDES_INSTALL_DIR}/apps/g2o_viewer)
