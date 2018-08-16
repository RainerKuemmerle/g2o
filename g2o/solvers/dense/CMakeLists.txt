add_library(solver_dense ${G2O_LIB_TYPE}
  solver_dense.cpp linear_solver_dense.h
)

set_target_properties(solver_dense PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_dense)
if (APPLE)
  set_target_properties(solver_dense PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()

target_link_libraries(solver_dense core)

install(TARGETS solver_dense
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/solvers/dense)
