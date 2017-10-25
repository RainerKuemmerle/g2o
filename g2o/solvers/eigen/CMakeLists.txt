ADD_LIBRARY(solver_eigen ${G2O_LIB_TYPE}
  solver_eigen.cpp
  linear_solver_eigen.h
)
SET_TARGET_PROPERTIES(solver_eigen PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_eigen)
TARGET_LINK_LIBRARIES(solver_eigen core)

SET(include_install_dir ${CMAKE_INSTALL_PREFIX}/include)

INSTALL(TARGETS solver_eigen
  EXPORT ${targets_export_name}
  RUNTIME DESTINATION ${runtime_destination}
  LIBRARY DESTINATION ${library_destination}
  ARCHIVE DESTINATION ${archive_destination}
  INCLUDES DESTINATION ${includes_destination}
)
FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${includes_destination}/g2o/solvers/eigen)
