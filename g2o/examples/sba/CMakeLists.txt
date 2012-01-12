ADD_EXECUTABLE(sba_demo
  sba_demo.cpp
)

SET_TARGET_PROPERTIES(sba_demo PROPERTIES OUTPUT_NAME sba_demo${EXE_POSTFIX})

INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(sba_demo core solver_cholmod types_icp types_sba)


