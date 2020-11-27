add_library(g2o_incremental_library ${G2O_LIB_TYPE}
  graph_optimizer_sparse_incremental.cpp graph_optimizer_sparse_incremental.h
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../)

set_target_properties(g2o_incremental_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}incremental)
target_link_libraries(g2o_incremental_library g2o_interactive_library solver_cholmod ${CAMD_LIBRARY})

add_executable(g2o_incremental_application g2o_incremental.cpp)
target_link_libraries(g2o_incremental_application g2o_incremental_library)
set_target_properties(g2o_incremental_application PROPERTIES OUTPUT_NAME g2o_incremental${EXE_POSTFIX})

install(TARGETS g2o_incremental_library g2o_incremental_application
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/examples/interactive_slam/g2o_incremental)
