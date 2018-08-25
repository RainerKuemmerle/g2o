set(interface_library_src
  abstract_slam_interface.h
  parser_interface.cpp
  parser_interface.h
  slam_context_interface.cpp
  slam_context_interface.h
)

if (MSVC)
  add_library(interface_library STATIC
    ${interface_library_src}
  )
else()
  add_library(interface_library ${G2O_LIB_TYPE}
    ${interface_library_src}
  )
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../)

set_target_properties(interface_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}interface)
target_link_libraries(interface_library parser_library)

install(TARGETS interface_library
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/examples/interactive_slam/slam_parser/interface)
