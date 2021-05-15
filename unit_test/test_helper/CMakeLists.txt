add_library(unittest_helper
  allocate_optimizer.cpp allocate_optimizer.h
)
target_link_libraries(unittest_helper solver_eigen)
target_include_directories(unittest_helper INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
