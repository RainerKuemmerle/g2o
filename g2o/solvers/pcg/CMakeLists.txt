add_library(solver_pcg ${G2O_LIB_TYPE}
  solver_pcg.cpp
  linear_solver_pcg.h
  linear_solver_pcg.hpp
)

set_target_properties(solver_pcg PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_pcg)
set_target_properties(solver_pcg PROPERTIES
  VERSION ${G2O_LIB_VERSION}
  SOVERSION ${G2O_LIB_SOVERSION})
if (APPLE)
  set_target_properties(solver_pcg PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

target_compile_features(solver_pcg PUBLIC cxx_std_17)
target_link_libraries(solver_pcg core)

target_include_directories(solver_pcg PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/g2o/solvers/pcg>)

install(TARGETS solver_pcg
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

# Create alias target for easier usage when included as subdirectory
add_library(g2o::solver_pcg ALIAS solver_pcg)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/solvers/pcg)
