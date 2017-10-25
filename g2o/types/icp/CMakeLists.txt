ADD_LIBRARY(types_icp ${G2O_LIB_TYPE}
  types_icp.cpp  types_icp.h
  g2o_types_icp_api.h
)

SET_TARGET_PROPERTIES(types_icp PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_icp)

TARGET_LINK_LIBRARIES(types_icp types_sba types_slam3d)

INSTALL(TARGETS types_icp
  EXPORT ${targets_export_name}
  RUNTIME DESTINATION ${runtime_destination}
  LIBRARY DESTINATION ${library_destination}
  ARCHIVE DESTINATION ${archive_destination}
  INCLUDES DESTINATION ${includes_destination}
  )

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${includes_destination}/g2o/types/icp)
