add_library(g2o_hierarchical_library ${G2O_LIB_TYPE}
  edge_labeler.cpp
  edge_creator.cpp
  star.cpp
  edge_types_cost_function.cpp
  backbone_tree_action.cpp
  simple_star_ops.cpp
  g2o_hierarchical_test_functions.cpp
  edge_labeler.h
  edge_creator.h
  star.h
  edge_types_cost_function.h
  backbone_tree_action.h
  simple_star_ops.h
  g2o_hierarchical_api.h
)


set_target_properties(g2o_hierarchical_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}hierarchical)
if (APPLE)
  set_target_properties(g2o_hierarchical_library PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

target_link_libraries(g2o_hierarchical_library core)

add_executable(g2o_hierarchical_application
  g2o_hierarchical.cpp)

target_link_libraries(g2o_hierarchical_application g2o_hierarchical_library g2o_cli_library types_slam3d ) #types_slam3d_new

set_target_properties(g2o_hierarchical_application PROPERTIES OUTPUT_NAME g2o_hierarchical${EXE_POSTFIX})

install(TARGETS g2o_hierarchical_library
   EXPORT ${G2O_TARGETS_EXPORT_NAME}
   RUNTIME DESTINATION ${RUNTIME_DESTINATION}
   LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES
  ${headers}
  DESTINATION ${INCLUDES_INSTALL_DIR}/apps/g2o_hierarchical)
