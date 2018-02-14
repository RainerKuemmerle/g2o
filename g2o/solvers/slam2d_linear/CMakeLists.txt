IF(NOT G2O_USE_CSPARSE)
  RETURN()
ENDIF()

ADD_LIBRARY(solver_slam2d_linear ${G2O_LIB_TYPE}
  slam2d_linear.cpp
  solver_slam2d_linear.h solver_slam2d_linear.cpp
  g2o_slam2d_linear_api.h
)

INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

SET_TARGET_PROPERTIES(solver_slam2d_linear PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_slam2d_linear)
TARGET_LINK_LIBRARIES(solver_slam2d_linear solver_csparse types_slam2d)

INSTALL(TARGETS solver_slam2d_linear
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION include/g2o/solvers/slam2d_linear)
