INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})

ADD_EXECUTABLE(static_target
  static_target.cpp
)
TARGET_LINK_LIBRARIES(static_target core solver_cholmod)
SET_TARGET_PROPERTIES(static_target PROPERTIES OUTPUT_NAME static_target${EXE_POSTFIX})

ADD_EXECUTABLE(constant_velocity_target
  constant_velocity_target.cpp
)

SET_TARGET_PROPERTIES(constant_velocity_target PROPERTIES OUTPUT_NAME constant_velocity_target${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(constant_velocity_target core solver_cholmod solver_pcg types_slam2d)


