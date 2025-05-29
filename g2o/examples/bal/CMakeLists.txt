add_executable(bal_example bal_example.cpp)

if(TARGET solver_cholmod)
  target_link_libraries(bal_example PRIVATE solver_cholmod)
else()
  target_link_libraries(bal_example PRIVATE solver_eigen)
endif()
target_link_libraries(bal_example PRIVATE CLI11::CLI11)
