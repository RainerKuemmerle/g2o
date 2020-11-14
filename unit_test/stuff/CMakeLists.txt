add_executable(unittest_stuff
  command_args_tests.cpp
  string_tools_tests.cpp
)
target_link_libraries(unittest_stuff stuff)
create_test(unittest_stuff)
