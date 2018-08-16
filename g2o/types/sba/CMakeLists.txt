add_library(types_sba ${G2O_LIB_TYPE}
  types_sba.h     types_six_dof_expmap.h
  types_sba.cpp   types_six_dof_expmap.cpp
  g2o_types_sba_api.h
)

set_target_properties(types_sba PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sba)
if (APPLE)
  set_target_properties(types_sba PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()
target_link_libraries(types_sba core types_slam3d)

install(TARGETS types_sba
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/types/sba)
