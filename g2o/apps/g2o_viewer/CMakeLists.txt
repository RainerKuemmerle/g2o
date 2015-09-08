# CMake: Do not link against qtmain.lib in Windows automatically
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

# Qt{4,5} have been find_package'd() by FindQGLViewer.cmake
if(Qt4_FOUND)
  INCLUDE(${QT_USE_FILE})

  QT4_WRAP_UI(UI_HEADERS base_main_window.ui base_properties_widget.ui)
  QT4_WRAP_CPP(UI_SOURCES main_window.h properties_widget.h)

  INCLUDE_DIRECTORIES(${QT_INCLUDES})
  SET(MY_QT_LIBRARIES ${QT_LIBRARIES})
endif()

if(Qt5_FOUND)
  QT5_WRAP_UI(UI_HEADERS base_main_window.ui base_properties_widget.ui)
  QT5_WRAP_CPP(UI_SOURCES main_window.h properties_widget.h)

  INCLUDE_DIRECTORIES(${Qt5Core_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
  SET(MY_QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Xml_LIBRARIES} ${Qt5OpenGL_LIBRARIES})
endif()

INCLUDE_DIRECTORIES(${QGLVIEWER_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_LIBRARY(viewer_library ${G2O_LIB_TYPE}
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
SET_TARGET_PROPERTIES(viewer_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}viewer)

ADD_EXECUTABLE(g2o_viewer
  g2o_viewer.cpp
)

SET_TARGET_PROPERTIES(g2o_viewer PROPERTIES OUTPUT_NAME g2o_viewer${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(viewer_library core g2o_cli_library ${QGLVIEWER_LIBRARY} ${MY_QT_LIBRARIES} ${OPENGL_LIBRARY})
TARGET_LINK_LIBRARIES(viewer_library core opengl_helper)
TARGET_LINK_LIBRARIES(g2o_viewer viewer_library)

INSTALL(TARGETS g2o_viewer viewer_library
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/apps/g2o_viewer)
INSTALL(FILES ${UI_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/apps/g2o_viewer)
