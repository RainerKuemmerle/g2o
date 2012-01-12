ADD_LIBRARY(example_library ${G2O_LIB_TYPE}
  example_slam_interface.cpp
  example_slam_interface.h
)

SET_TARGET_PROPERTIES(example_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}example)

TARGET_LINK_LIBRARIES(example_library parser_library interface_library)

ADD_EXECUTABLE(test_slam_interface test_slam_interface.cpp)
TARGET_LINK_LIBRARIES(test_slam_interface example_library)
SET_TARGET_PROPERTIES(test_slam_interface PROPERTIES OUTPUT_NAME test_slam_interface${EXE_POSTFIX})

#INSTALL(TARGETS example_library
  #RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  #LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  #ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#)

#FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
#INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/slam_parser/example)
