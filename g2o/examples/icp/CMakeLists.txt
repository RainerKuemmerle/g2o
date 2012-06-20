ADD_EXECUTABLE(gicp_demo 
  gicp_demo.cpp
)

ADD_EXECUTABLE(gicp_sba_demo 
  gicp_sba_demo.cpp
)

SET_TARGET_PROPERTIES(gicp_sba_demo PROPERTIES OUTPUT_NAME gicp_sba_demo${EXE_POSTFIX})

INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(gicp_demo core types_sba types_slam3d types_icp ${OPENGL_LIBRARIES} solver_csparse)
TARGET_LINK_LIBRARIES(gicp_sba_demo core types_sba types_slam3d types_icp ${OPENGL_LIBRARIES} solver_csparse)

