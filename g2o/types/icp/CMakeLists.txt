add_library(types_icp ${G2O_LIB_TYPE}
  types_icp.cpp  types_icp.h
  g2o_types_icp_api.h
)

set_target_properties(types_icp PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_icp)
if (APPLE)
  set_target_properties(types_icp PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()
target_link_libraries(types_icp types_sba types_slam3d)

install(TARGETS types_icp
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
  )

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/types/icp)
