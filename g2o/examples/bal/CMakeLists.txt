if(CHOLMOD_INCLUDE_DIR OR CSPARSE_INCLUDE_DIR)
  add_executable(bal_example bal_example.cpp)
endif()

# Link CHOLMOD if available, CSparse otherwise
if(CHOLMOD_FOUND)
  include_directories(${CHOLMOD_INCLUDE_DIR})
  target_link_libraries(bal_example solver_cholmod)
elseif(CSPARSE_FOUND)
  include_directories(${CSPARSE_INCLUDE_DIR})
  target_link_libraries(bal_example solver_csparse)
endif()
