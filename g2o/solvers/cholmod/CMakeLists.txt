if (NOT G2O_USE_CHOLMOD)
  return()
endif()

set(PUBLIC_HEADERS
	linear_solver_cholmod.h
	cholmod_wrapper.h
	g2o_cholmod_api.h
)

add_library(solver_cholmod ${G2O_LIB_TYPE}
  cholmod_wrapper.cpp
  solver_cholmod.cpp
)

target_include_directories(solver_cholmod PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/g2o/solvers/cholmod>)

set_target_properties(solver_cholmod PROPERTIES
    OUTPUT_NAME ${LIB_PREFIX}solver_cholmod
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
)
set_target_properties(solver_cholmod PROPERTIES
  VERSION ${G2O_LIB_VERSION}
  SOVERSION ${G2O_LIB_SOVERSION})

if (APPLE)
  set_target_properties(solver_cholmod PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

add_definitions(${BLAS_DEFINITIONS} ${LAPACK_DEFINITIONS})
target_compile_features(solver_cholmod PUBLIC cxx_std_17)
target_link_libraries(solver_cholmod
    PUBLIC core
    PRIVATE SuiteSparse::CHOLMOD
)

install(TARGETS solver_cholmod
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  PUBLIC_HEADER DESTINATION ${INCLUDES_DESTINATION}/g2o/solvers/cholmod
)

# Create alias target for easier usage when included as subdirectory
add_library(g2o::solver_cholmod ALIAS solver_cholmod)
