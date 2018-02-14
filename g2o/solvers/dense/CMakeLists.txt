ADD_LIBRARY(solver_dense ${G2O_LIB_TYPE}
  solver_dense.cpp linear_solver_dense.h
)

SET_TARGET_PROPERTIES(solver_dense PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_dense)

TARGET_LINK_LIBRARIES(solver_dense core)

INSTALL(TARGETS solver_dense
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION include/g2o/solvers/dense)
