ADD_EXECUTABLE(bal_example
  bal_example.cpp
)

# Link CHOLMOD if available, CSparse otherwise
IF(CHOLMOD_FOUND)
  INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(bal_example solver_cholmod)
ELSE()
  INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(bal_example solver_csparse)
ENDIF()
