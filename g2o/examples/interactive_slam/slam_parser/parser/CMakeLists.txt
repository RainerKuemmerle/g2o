SET(G2O_PARSER_SRC
  slam_context.h slam_context.cpp
  bison_parser.cpp bison_parser.h
  commands.h
  driver.h driver.cpp
  location.hh
  position.hh
  scanner.h flex_scanner.cpp
  stack.hh
)

IF (MSVC)
ADD_LIBRARY(parser_library STATIC
  ${G2O_PARSER_SRC}
)
ELSE()
ADD_LIBRARY(parser_library ${G2O_LIB_TYPE}
  ${G2O_PARSER_SRC}
)
ENDIF()

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET_TARGET_PROPERTIES(parser_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}parser)

#ADD_EXECUTABLE(test_slam_parser test_slam_parser.cpp)
#TARGET_LINK_LIBRARIES(test_slam_parser parser_library)
#SET_TARGET_PROPERTIES(test_slam_parser PROPERTIES OUTPUT_NAME test_slam_parser${EXE_POSTFIX})

INSTALL(TARGETS parser_library
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hh")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/slam_parser/parser)
