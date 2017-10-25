ADD_LIBRARY(solver_structure_only ${G2O_LIB_TYPE}
  structure_only.cpp
  structure_only_solver.h
)

SET_TARGET_PROPERTIES(solver_structure_only PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_structure_only)
TARGET_LINK_LIBRARIES(solver_structure_only core)
SET(include_install_dir ${CMAKE_INSTALL_PREFIX}/include)

INSTALL(TARGETS solver_structure_only
  EXPORT ${targets_export_name}
  RUNTIME DESTINATION ${runtime_destination}
  LIBRARY DESTINATION ${library_destination}
  ARCHIVE DESTINATION ${archive_destination}
  INCLUDES DESTINATION ${includes_destination}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${includes_destination}/g2o/solvers/structure_only)
