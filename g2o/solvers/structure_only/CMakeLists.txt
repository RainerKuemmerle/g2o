ADD_LIBRARY(solver_structure_only ${G2O_LIB_TYPE}
  structure_only.cpp
  structure_only_solver.h
)

SET_TARGET_PROPERTIES(solver_structure_only PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_structure_only)
TARGET_LINK_LIBRARIES(solver_structure_only core)

INSTALL(TARGETS solver_structure_only
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION include/g2o/solvers/structure_only)
