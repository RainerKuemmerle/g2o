# Qt5 was been find_package'd() by FindQGLViewer.cmake
if(Qt5_FOUND)
  QT5_WRAP_UI(UI_HEADERS base_main_window.ui base_properties_widget.ui)
  QT5_WRAP_CPP(UI_SOURCES main_window.h properties_widget.h)

  include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
  string(APPEND qt5_includes_dirs "${Qt5Core_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS}")
  set(MY_QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Xml_LIBRARIES} ${Qt5OpenGL_LIBRARIES})
endif()

include_directories(SYSTEM ${QGLVIEWER_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_library(viewer_library ${G2O_LIB_TYPE}
  g2o_viewer_api.h
  g2o_qglviewer.cpp g2o_qglviewer.h
  main_window.cpp main_window.h
  stream_redirect.cpp stream_redirect.h
  gui_hyper_graph_action.cpp gui_hyper_graph_action.h
  properties_widget.cpp properties_widget.h
  viewer_properties_widget.cpp viewer_properties_widget.h
  run_g2o_viewer.cpp run_g2o_viewer.h
  ${UI_HEADERS} ${UI_SOURCES}
)
set_target_properties(viewer_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}viewer)
if (APPLE)
  set_target_properties(viewer_library PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

add_executable(g2o_viewer
  g2o_viewer.cpp
)

set_target_properties(g2o_viewer PROPERTIES OUTPUT_NAME g2o_viewer${EXE_POSTFIX})

target_link_libraries(viewer_library core g2o_cli_library ${QGLVIEWER_LIBRARY} ${MY_QT_LIBRARIES} ${OPENGL_LIBRARY})
target_link_libraries(viewer_library core opengl_helper)
target_link_libraries(g2o_viewer viewer_library)

install(TARGETS g2o_viewer viewer_library
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION} ${qt5_includes_dirs} ${QGLVIEWER_INCLUDE_DIR}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/apps/g2o_viewer)
install(FILES ${UI_HEADERS} DESTINATION ${INCLUDES_INSTALL_DIR}/apps/g2o_viewer)
