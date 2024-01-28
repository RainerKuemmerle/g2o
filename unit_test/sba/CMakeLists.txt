add_executable(unittest_sba
sba_basic_tests.cpp
)
target_link_libraries(unittest_sba unittest_helper types_sba)
create_test(unittest_sba)
