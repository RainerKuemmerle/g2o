if (NOT G2O_USE_CHOLMOD)
  return()
endif()

add_library(solver_cholmod ${G2O_LIB_TYPE}
  solver_cholmod.cpp
  linear_solver_cholmod.h
)

if (NOT TARGET SuiteSparse::cholmod)
  include_directories(${CHOLMOD_INCLUDE_DIR})
   add_definitions(${BLAS_DEFINITIONS} ${LAPACK_DEFINITIONS})
endif()

set_target_properties(solver_cholmod PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_cholmod)
if (APPLE)
  set_target_properties(solver_cholmod PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

target_link_libraries(solver_cholmod ${G2O_CHOLMOD_TARGET} core)

install(TARGETS solver_cholmod
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION} ${CHOLMOD_INCLUDE_DIR}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/solvers/cholmod)
