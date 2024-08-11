add_library(unittest_helper ${G2O_LIB_TYPE}
  utils.cpp
  allocate_optimizer.cpp allocate_optimizer.h
)
if (BUILD_SHARED_LIBS)
  set_target_properties(unittest_helper PROPERTIES COMPILE_DEFINITIONS "GTEST_LINKED_AS_SHARED_LIBRARY=1")
endif()
target_link_libraries(unittest_helper solver_eigen stuff gmock)
