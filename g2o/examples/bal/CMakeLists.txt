if(CHOLMOD_FOUND OR CSPARSE_FOUND)
  add_executable(bal_example bal_example.cpp)
endif()

# Link CHOLMOD if available, CSparse otherwise
if(CHOLMOD_FOUND)
  target_link_libraries(bal_example solver_cholmod)
elseif(CSPARSE_FOUND)
  target_link_libraries(bal_example solver_csparse)
endif()
