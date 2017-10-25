ADD_LIBRARY(solver_pcg ${G2O_LIB_TYPE}
  solver_pcg.cpp
  linear_solver_pcg.h  
  linear_solver_pcg.hpp
)

SET_TARGET_PROPERTIES(solver_pcg PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_pcg)

TARGET_LINK_LIBRARIES(solver_pcg core)

SET(include_install_dir ${CMAKE_INSTALL_PREFIX}/include)

INSTALL(TARGETS solver_pcg
  EXPORT ${targets_export_name}
  RUNTIME DESTINATION ${runtime_destination}
  LIBRARY DESTINATION ${library_destination}
  ARCHIVE DESTINATION ${archive_destination}
  INCLUDES DESTINATION ${includes_destination}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${includes_destination}/g2o/solvers/pcg)
