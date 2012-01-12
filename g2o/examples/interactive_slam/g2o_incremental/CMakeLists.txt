ADD_LIBRARY(g2o_incremental_library ${G2O_LIB_TYPE}
  graph_optimizer_sparse_incremental.cpp graph_optimizer_sparse_incremental.h
)

INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../)

SET_TARGET_PROPERTIES(g2o_incremental_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}incremental)
TARGET_LINK_LIBRARIES(g2o_incremental_library g2o_interactive_library solver_cholmod ${CAMD_LIBRARY})

ADD_EXECUTABLE(g2o_incremental_application g2o_incremental.cpp)
TARGET_LINK_LIBRARIES(g2o_incremental_application g2o_incremental_library)
SET_TARGET_PROPERTIES(g2o_incremental_application PROPERTIES OUTPUT_NAME g2o_incremental${EXE_POSTFIX})

INSTALL(TARGETS g2o_incremental_library g2o_incremental_application
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/examples/interactive_slam/g2o_incremental)
