set(G2O_PARSER_SRC
  slam_context.h slam_context.cpp
  bison_parser.cpp bison_parser.h
  commands.h
  driver.h driver.cpp
  location.hh
  position.hh
  scanner.h flex_scanner.cpp
  stack.hh
)

if (MSVC)
add_library(parser_library STATIC
  ${G2O_PARSER_SRC}
)
else()
add_library(parser_library ${G2O_LIB_TYPE}
  ${G2O_PARSER_SRC}
)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(parser_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}parser)

#add_executable(test_slam_parser test_slam_parser.cpp)
#target_link_libraries(test_slam_parser parser_library)
#set_target_properties(test_slam_parser PROPERTIES OUTPUT_NAME test_slam_parser${EXE_POSTFIX})

install(TARGETS parser_library
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hh")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/examples/interactive_slam/slam_parser/parser)
