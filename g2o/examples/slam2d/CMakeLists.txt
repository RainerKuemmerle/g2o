# qt components used (also by qglviewer): Core Gui Xml OpenGL Widgets

INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR}
    ${QGLVIEWER_INCLUDE_DIR}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
    ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
    )

QT5_WRAP_UI(UI_HEADERS base_main_window.ui)
QT5_WRAP_CPP(UI_SOURCES main_window.h)

ADD_EXECUTABLE(slam2d_g2o
  main_window.cpp
  slam2d_viewer.cpp
  slam2d_viewer.h
  slam2d_g2o.cpp
  draw_helpers.cpp
  ${UI_HEADERS}
  ${UI_SOURCES}
)

SET_TARGET_PROPERTIES(slam2d_g2o PROPERTIES OUTPUT_NAME slam2d_g2o${EXE_POSTFIX})

IF(Qt5_POSITION_INDEPENDENT_CODE)
    SET_PROPERTY(TARGET slam2d_g2o PROPERTY COMPILE_FLAGS -fPIC)
    MESSAGE(STATUS "Generating position indpendent code for slam2d because Qt5 was built with -reduce-relocations")
    # Note: using
    #       SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
    #       does not seem to work: This generates some libraries with -fPIE which is not enough for Qt...
ENDIF()




TARGET_LINK_LIBRARIES(slam2d_g2o core solver_csparse types_slam2d
    ${QGLVIEWER_LIBRARY}
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5Xml_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5OpenGL_LIBRARIES}
    ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

