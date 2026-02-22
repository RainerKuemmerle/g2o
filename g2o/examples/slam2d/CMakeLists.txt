include_directories(${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Qt6 COMPONENTS Widgets OpenGLWidgets REQUIRED)

# Use automoc/autouic so Qt6's moc/uic are used
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

set(UI_HEADERS base_main_window.ui)
set(UI_SOURCES main_window.h)

add_executable(slam2d_g2o
  main_window.cpp
  slam2d_viewer.cpp
  slam2d_viewer.h
  slam2d_g2o.cpp
  ${UI_HEADERS}
  ${UI_SOURCES}
)

set_target_properties(slam2d_g2o PROPERTIES OUTPUT_NAME slam2d_g2o)

if(Qt6_POSITION_INDEPENDENT_CODE)
  set_property(TARGET slam2d_g2o PROPERTY COMPILE_FLAGS -fPIC)
  message(STATUS "Generating position independent code for slam2d because Qt6 was built with -reduce-relocations")
  # Note: using
  #       set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  #       does not seem to work: This generates some libraries with -fPIE which is not enough for Qt...
endif()

target_link_libraries(slam2d_g2o core solver_eigen types_slam2d opengl_helper
  viewer_library
  Qt6::Widgets Qt6::OpenGLWidgets
)
