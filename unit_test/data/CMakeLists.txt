add_executable(unittest_data
  data_queue_tests.cpp
  io_data.cpp
)
target_link_libraries(unittest_data types_data stuff)
create_test(unittest_data)
