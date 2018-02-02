ADD_LIBRARY(types_sba ${G2O_LIB_TYPE}
  types_sba.h     types_six_dof_expmap.h
  types_sba.cpp   types_six_dof_expmap.cpp
  g2o_types_sba_api.h
)

SET_TARGET_PROPERTIES(types_sba PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sba)

TARGET_LINK_LIBRARIES(types_sba core types_slam3d)

INSTALL(TARGETS types_sba
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION include/g2o/types/sba)
