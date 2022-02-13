add_library(unittest_helper ${G2O_LIB_TYPE}
  allocate_optimizer.cpp allocate_optimizer.h
)
target_link_libraries(unittest_helper solver_eigen)
