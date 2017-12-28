IF(NOT G2O_USE_CSPARSE)
  RETURN()
ENDIF()

INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY(csparse_extension ${G2O_LGPL_LIB_TYPE}
  csparse_helper.cpp csparse_helper.h
  g2o_csparse_extension_api.h
)
SET_TARGET_PROPERTIES(csparse_extension PROPERTIES OUTPUT_NAME ${LIB_PREFIX}csparse_extension)

IF(BUILD_CSPARSE)
  TARGET_LINK_LIBRARIES(csparse_extension csparse)
ELSE()
  TARGET_LINK_LIBRARIES(csparse_extension ${CSPARSE_LIBRARY})
ENDIF()

TARGET_LINK_LIBRARIES(csparse_extension ${G2O_EIGEN3_EIGEN_TARGET})

SET(include_install_dir ${CMAKE_INSTALL_PREFIX}/include)

ADD_LIBRARY(solver_csparse ${G2O_LIB_TYPE}
  solver_csparse.cpp
  linear_solver_csparse.h
  g2o_csparse_api.h
)
SET_TARGET_PROPERTIES(solver_csparse PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_csparse)

TARGET_LINK_LIBRARIES(solver_csparse csparse_extension core)


INSTALL(TARGETS solver_csparse csparse_extension
  EXPORT ${targets_export_name}
  RUNTIME DESTINATION ${runtime_destination}
  LIBRARY DESTINATION ${library_destination}
  ARCHIVE DESTINATION ${archive_destination}
  INCLUDES DESTINATION ${includes_destination}
)
FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${includes_destination}/g2o/solvers/csparse)
