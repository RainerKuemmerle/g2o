cmake_minimum_required(VERSION 2.6)

PROJECT(csparse)

SET(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")

ADD_LIBRARY(csparse ${G2O_LGPL_LIB_TYPE}
  cs_add.c
  cs_amd.c
  cs_chol.c
  cs_cholsol.c
  cs_compress.c
  cs_counts.c
  cs_cumsum.c
  cs_dfs.c
  cs_dmperm.c
  cs_droptol.c
  cs_dropzeros.c
  cs_dupl.c
  cs_entry.c
  cs_ereach.c
  cs_etree.c
  cs_fkeep.c
  cs_gaxpy.c
  cs_happly.c
  cs_house.c
  cs_ipvec.c
  cs_leaf.c
  cs_load.c
  cs_lsolve.c
  cs_ltsolve.c
  cs_lu.c
  cs_lusol.c
  cs_malloc.c
  cs_maxtrans.c
  cs_multiply.c
  cs_norm.c
  cs_permute.c
  cs_pinv.c
  cs_post.c
  cs_print.c
  cs_pvec.c
  cs_qr.c
  cs_qrsol.c
  cs_randperm.c
  cs_reach.c
  cs_scatter.c
  cs_scc.c
  cs_schol.c
  cs_spsolve.c
  cs_sqr.c
  cs_symperm.c
  cs_tdfs.c
  cs_transpose.c
  cs_updown.c
  cs_usolve.c
  cs_util.c
  cs_utsolve.c
  cs_api.h
)

SET_TARGET_PROPERTIES(csparse PROPERTIES OUTPUT_NAME ${LIB_PREFIX}ext_csparse)
IF (UNIX)
  TARGET_LINK_LIBRARIES(csparse m)
ENDIF()

INSTALL(TARGETS csparse
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/EXTERNAL/csparse)

# Set up the variables
if(POLICY CMP0026)
  cmake_policy(SET CMP0026 OLD)
endif()
GET_TARGET_PROPERTY(CSPARSE_LIBRARY csparse LOCATION)
SET(CSPARSE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Include directory for CSparse" FORCE)
SET(CSPARSE_LIBRARY     ${CSPARSE_LIBRARY} CACHE FILEPATH "CSparse library" FORCE)
#SET(CSPARSE_FOUND TRUE PARENT_SCOPE)

#MESSAGE("CSPARSE_LIBRARY = ${CSPARSE_LIBRARY}")
#MESSAGE("CSPARSE_INCLUDE_DIR = ${CSPARSE_INCLUDE_DIR}")
