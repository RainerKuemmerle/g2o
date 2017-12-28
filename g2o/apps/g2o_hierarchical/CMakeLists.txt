ADD_LIBRARY(g2o_hierarchical_library ${G2O_LIB_TYPE}
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


SET_TARGET_PROPERTIES(g2o_hierarchical_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}hierarchical)

TARGET_LINK_LIBRARIES(g2o_hierarchical_library core)

ADD_EXECUTABLE(g2o_hierarchical_application
  g2o_hierarchical.cpp)

TARGET_LINK_LIBRARIES(g2o_hierarchical_application g2o_hierarchical_library g2o_cli_library types_slam3d ) #types_slam3d_new

SET_TARGET_PROPERTIES(g2o_hierarchical_application PROPERTIES OUTPUT_NAME g2o_hierarchical${EXE_POSTFIX})

# INSTALL(TARGETS g2o_hierarchical_library g2o_cli_application
#   EXPORT ${G2O_TARGETS_EXPORT_NAME}
#   RUNTIME DESTINATION ${RUNTIME_DESTINATION}
#   LIBRARY DESTINATION ${LIBRARY_DESTINATION}
#   ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
# )

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES
  ${headers}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/apps/g2o_hierarchical)
