add_executable(bal_example bal_example.cpp)

# Link CHOLMOD if available, CSparse otherwise
if(CHOLMOD_FOUND)
  target_link_libraries(bal_example solver_cholmod)
else()
  target_link_libraries(bal_example solver_eigen)
endif()
