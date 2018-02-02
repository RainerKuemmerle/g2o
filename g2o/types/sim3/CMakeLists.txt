ADD_LIBRARY(types_sim3 ${G2O_LIB_TYPE}
  types_seven_dof_expmap.cpp
  types_seven_dof_expmap.h
)

SET_TARGET_PROPERTIES(types_sim3 PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sim3)

TARGET_LINK_LIBRARIES(types_sim3 types_sba)

INSTALL(TARGETS types_sim3
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION include/g2o/types/sim3)

