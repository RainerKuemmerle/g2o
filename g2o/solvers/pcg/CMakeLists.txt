ADD_LIBRARY(solver_pcg ${G2O_LIB_TYPE}
  solver_pcg.cpp
  linear_solver_pcg.h  
  linear_solver_pcg.hpp
)

SET_TARGET_PROPERTIES(solver_pcg PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_pcg)

TARGET_LINK_LIBRARIES(solver_pcg core)

INSTALL(TARGETS solver_pcg
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION include/g2o/solvers/pcg)
