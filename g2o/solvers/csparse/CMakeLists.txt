if(NOT G2O_USE_CSPARSE)
  return()
endif()

add_library(csparse_extension ${G2O_LGPL_LIB_TYPE}
  csparse_helper.cpp csparse_helper.h
  csparse_extension.cpp csparse_extension.h
  g2o_csparse_extension_api.h
)
set_target_properties(csparse_extension PROPERTIES OUTPUT_NAME ${LIB_PREFIX}csparse_extension)
if (APPLE)
  set_target_properties(csparse_extension PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

target_include_directories(csparse_extension PUBLIC
  $<BUILD_INTERFACE:${CSPARSE_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/g2o/solvers/csparse>)
target_link_libraries(csparse_extension PUBLIC stuff ${CSPARSE_LIBRARY} ${G2O_EIGEN3_EIGEN_TARGET})

add_library(solver_csparse ${G2O_LIB_TYPE}
  solver_csparse.cpp
  linear_solver_csparse.h
  g2o_csparse_api.h
)
set_target_properties(solver_csparse PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_csparse)
if (APPLE)
  set_target_properties(solver_csparse PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

target_include_directories(solver_csparse PUBLIC
  $<BUILD_INTERFACE:${CSPARSE_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/g2o/solvers/csparse>)
target_link_libraries(solver_csparse
  PUBLIC csparse_extension
  PUBLIC core)

install(TARGETS solver_csparse csparse_extension
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
  )

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/solvers/csparse)
