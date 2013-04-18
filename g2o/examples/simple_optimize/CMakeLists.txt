INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_EXECUTABLE(simple_optimize
  simple_optimize.cpp
)

SET_TARGET_PROPERTIES(simple_optimize PROPERTIES OUTPUT_NAME simple_optimize${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(simple_optimize core solver_csparse)
TARGET_LINK_LIBRARIES(simple_optimize types_slam2d)
TARGET_LINK_LIBRARIES(simple_optimize types_slam3d)
