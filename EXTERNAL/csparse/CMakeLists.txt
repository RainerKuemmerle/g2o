cmake_minimum_required(VERSION 2.6)

project(csparse)

#set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")

add_library(csparse ${G2O_LGPL_LIB_TYPE}
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

set_target_properties(csparse PROPERTIES OUTPUT_NAME ${LIB_PREFIX}ext_csparse)
if (APPLE)
  set_target_properties(csparse PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()
if (UNIX)
  target_link_libraries(csparse PUBLIC m)
endif()

install(TARGETS csparse
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/EXTERNAL/csparse)

# Set up the variables
set(CSPARSE_LIBRARY "$<TARGET_FILE:csparse>")
set(CSPARSE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Include directory for CSparse" FORCE)
set(CSPARSE_LIBRARY     ${CSPARSE_LIBRARY} CACHE FILEPATH "CSparse library" FORCE)
