add_library(solver_eigen ${G2O_LIB_TYPE}
  solver_eigen.cpp
  linear_solver_eigen.h
)
set_target_properties(solver_eigen PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_eigen)
set_target_properties(solver_eigen PROPERTIES
  VERSION ${G2O_LIB_VERSION}
  SOVERSION ${G2O_LIB_SOVERSION})
if (APPLE)
  set_target_properties(solver_eigen PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()
target_compile_features(solver_eigen PUBLIC cxx_std_17)
target_link_libraries(solver_eigen core)

target_include_directories(solver_eigen PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/g2o/solvers/eigen>
)

install(TARGETS solver_eigen
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

# Create alias target for easier usage when included as subdirectory
add_library(g2o::solver_eigen ALIAS solver_eigen)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/solvers/eigen)
