SET(interface_library_src
  abstract_slam_interface.h
  parser_interface.cpp
  parser_interface.h
  slam_context_interface.cpp
  slam_context_interface.h
)

IF (MSVC)
  ADD_LIBRARY(interface_library STATIC
    ${interface_library_src}
  )
ELSE()
  ADD_LIBRARY(interface_library ${G2O_LIB_TYPE}
    ${interface_library_src}
  )
ENDIF()

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../)

SET_TARGET_PROPERTIES(interface_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}interface)
TARGET_LINK_LIBRARIES(interface_library parser_library)

INSTALL(TARGETS interface_library
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/slam_parser/interface)
