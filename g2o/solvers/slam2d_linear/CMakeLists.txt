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

SET(include_install_dir ${CMAKE_INSTALL_PREFIX}/include)

INSTALL(TARGETS solver_slam2d_linear
  EXPORT ${targets_export_name}
  RUNTIME DESTINATION ${runtime_destination}
  LIBRARY DESTINATION ${library_destination}
  ARCHIVE DESTINATION ${archive_destination}
  INCLUDES DESTINATION ${includes_destination} ${CSPARSE_INCLUDE_DIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${includes_destination}/g2o/solvers/slam2d_linear)
