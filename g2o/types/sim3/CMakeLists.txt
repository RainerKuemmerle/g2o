ADD_LIBRARY(types_sim3 ${G2O_LIB_TYPE}
  types_seven_dof_expmap.cpp
  types_seven_dof_expmap.h
)

SET_TARGET_PROPERTIES(types_sim3 PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sim3)

TARGET_LINK_LIBRARIES(types_sim3 types_sba)

INSTALL(TARGETS types_sim3
  EXPORT ${targets_export_name}
  RUNTIME DESTINATION ${runtime_destination}
  LIBRARY DESTINATION ${library_destination}
  ARCHIVE DESTINATION ${archive_destination}
  INCLUDES DESTINATION ${includes_destination}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${includes_destination}/g2o/types/sim3)

