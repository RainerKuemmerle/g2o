ADD_EXECUTABLE(sba_demo
  sba_demo.cpp
)

SET_TARGET_PROPERTIES(sba_demo PROPERTIES OUTPUT_NAME sba_demo${EXE_POSTFIX})

# Link CHOLMOD if available, CSparse otherwise
IF(CHOLMOD_FOUND)
  INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(sba_demo solver_cholmod)
ELSE()
  INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(sba_demo solver_csparse)
ENDIF()

TARGET_LINK_LIBRARIES(sba_demo core types_icp types_sba)
