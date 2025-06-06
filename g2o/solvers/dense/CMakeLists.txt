add_library(solver_dense ${G2O_LIB_TYPE}
  solver_dense.cpp linear_solver_dense.h
)

set_target_properties(solver_dense PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_dense)
set_target_properties(solver_dense PROPERTIES
  VERSION ${G2O_LIB_VERSION}
  SOVERSION ${G2O_LIB_SOVERSION})
if (APPLE)
  set_target_properties(solver_dense PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

target_compile_features(solver_dense PUBLIC cxx_std_17)
target_link_libraries(solver_dense core)

install(TARGETS solver_dense
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

# Create alias target for easier usage when included as subdirectory
add_library(g2o::solver_dense ALIAS solver_dense)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/solvers/dense)
