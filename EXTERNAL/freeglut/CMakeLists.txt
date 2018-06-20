ADD_LIBRARY(freeglut_minimal ${G2O_LIB_TYPE}
  freeglut_font.cpp
  freeglut_stroke_mono_roman.cpp
  freeglut_stroke_roman.cpp
)

TARGET_LINK_LIBRARIES(freeglut_minimal PUBLIC "${OPENGL_gl_LIBRARY}")
TARGET_LINK_LIBRARIES(freeglut_minimal PUBLIC "${LIB_PREFIX}config")

SET_TARGET_PROPERTIES(freeglut_minimal PROPERTIES OUTPUT_NAME ${LIB_PREFIX}ext_freeglut_minimal)
if (APPLE)
  set_target_properties(freeglut_minimal PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()

INSTALL(TARGETS freeglut_minimal
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION include/g2o/freeglut)
