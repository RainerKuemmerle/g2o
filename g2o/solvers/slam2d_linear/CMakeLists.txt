add_library(solver_slam2d_linear ${G2O_LIB_TYPE}
  slam2d_linear.cpp
  solver_slam2d_linear.h solver_slam2d_linear.cpp
  g2o_slam2d_linear_api.h
)

set_target_properties(solver_slam2d_linear PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_slam2d_linear)
set_target_properties(solver_slam2d_linear PROPERTIES
  VERSION ${G2O_LIB_VERSION}
  SOVERSION ${G2O_LIB_SOVERSION})
if (APPLE)
  set_target_properties(solver_slam2d_linear PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

target_compile_features(solver_slam2d_linear PUBLIC cxx_std_17)
target_link_libraries(solver_slam2d_linear solver_eigen types_slam2d)

install(TARGETS solver_slam2d_linear
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

# Create alias target for easier usage when included as subdirectory
add_library(g2o::solver_slam2d_linear ALIAS solver_slam2d_linear)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/solvers/slam2d_linear)
