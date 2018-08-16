add_library(example_library ${G2O_LIB_TYPE}
  example_slam_interface.cpp
  example_slam_interface.h
)

set_target_properties(example_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}example)

target_link_libraries(example_library parser_library interface_library)

add_executable(test_slam_interface test_slam_interface.cpp)
target_link_libraries(test_slam_interface example_library)
set_target_properties(test_slam_interface PROPERTIES OUTPUT_NAME test_slam_interface${EXE_POSTFIX})

#install(TARGETS example_library
  #RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  #LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  #ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
#)

#file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
#install(FILES ${headers} DESTINATION include/slam_parser/example)
