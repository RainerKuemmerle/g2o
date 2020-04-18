add_library(g2o_cli_library ${G2O_LIB_TYPE}
  dl_wrapper.cpp dl_wrapper.h
  output_helper.cpp output_helper.h
  g2o_common.cpp g2o_common.h
  g2o_cli_api.h
)

set_target_properties(g2o_cli_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}cli)
if (APPLE)
  set_target_properties(g2o_cli_library PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()

set_property(TARGET g2o_cli_library APPEND PROPERTY COMPILE_DEFINITIONS G2O_DEFAULT_TYPES_DIR_="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" G2O_DEFAULT_SOLVERS_DIR_="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")

target_link_libraries(g2o_cli_library core)
if(UNIX)
  target_link_libraries(g2o_cli_library ${CMAKE_DL_LIBS})
endif(UNIX)

add_executable(g2o_cli_application
  g2o.cpp)

target_link_libraries(g2o_cli_application g2o_cli_library)

set_property(TARGET g2o_cli_library APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Debug>:G2O_LIBRARY_POSTFIX="${CMAKE_DEBUG_POSTFIX}">)
set_property(TARGET g2o_cli_library APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:RelWithDebInfo>:G2O_LIBRARY_POSTFIX="${CMAKE_RELWITHDEBINFO_POSTFIX}">)
set_property(TARGET g2o_cli_library APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:MinSizeRel>:G2O_LIBRARY_POSTFIX="${CMAKE_MINSIZEREL_POSTFIX}">)

#if(POLICY CMP0043)
#cmake_policy(SET CMP0043 OLD)
#endif()
#set_property(TARGET g2o_cli_library APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG          G2O_LIBRARY_POSTFIX="${CMAKE_DEBUG_POSTFIX}")
#set_property(TARGET g2o_cli_library APPEND PROPERTY COMPILE_DEFINITIONS_RELWITHDEBINFO G2O_LIBRARY_POSTFIX="${CMAKE_RELWITHDEBINFO_POSTFIX}")
#set_property(TARGET g2o_cli_library APPEND PROPERTY COMPILE_DEFINITIONS_MINSIZEREL     G2O_LIBRARY_POSTFIX="${CMAKE_MINSIZEREL_POSTFIX}")

set_target_properties(g2o_cli_application PROPERTIES OUTPUT_NAME g2o${EXE_POSTFIX})

install(TARGETS g2o_cli_library g2o_cli_application
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/apps/g2o_cli)
