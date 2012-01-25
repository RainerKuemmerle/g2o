INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY(tutorial_slam2d_library ${G2O_LIB_TYPE}
  edge_se2_pointxy.cpp  se2.h                vertex_point_xy.h
  edge_se2.cpp    edge_se2_pointxy.h    vertex_se2.cpp
  edge_se2.h      rand.h                vertex_point_xy.cpp  vertex_se2.h
  parameter_se2_offset.h parameter_se2_offset.cpp
  types_tutorial_slam2d.h types_tutorial_slam2d.cpp
  simulator.h simulator.cpp
  g2o_tutorial_slam2d_api.h
)

SET_TARGET_PROPERTIES(tutorial_slam2d_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}tutorial_slam2d)

TARGET_LINK_LIBRARIES(tutorial_slam2d_library core solver_csparse)

ADD_EXECUTABLE(tutorial_slam2d
  tutorial_slam2d.cpp
)

SET_TARGET_PROPERTIES(tutorial_slam2d PROPERTIES OUTPUT_NAME tutorial_slam2d${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(tutorial_slam2d tutorial_slam2d_library)
