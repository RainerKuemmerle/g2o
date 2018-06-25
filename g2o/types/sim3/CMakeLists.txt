ADD_LIBRARY(types_sim3 ${G2O_LIB_TYPE}
  types_seven_dof_expmap.cpp
  types_seven_dof_expmap.h
)

SET_TARGET_PROPERTIES(types_sim3 PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sim3)
if (APPLE)
  set_target_properties(types_sim3 PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()
TARGET_LINK_LIBRARIES(types_sim3 PUBLIC types_sba)
TARGET_INCLUDE_DIRECTORIES(types_sim3 PUBLIC "${g2o_SOURCE_DIR}")

INSTALL(TARGETS types_sim3
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION include/g2o/types/sim3)

