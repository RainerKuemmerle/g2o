set_source_files_properties(${PROTO_SRCS} PROPERTIES GENERATED TRUE)
add_library(graph_proto ${G2O_LIB_TYPE} ${PROTO_SRCS})
add_dependencies(graph_proto graph_proto_dependency)

set_target_properties(graph_proto PROPERTIES OUTPUT_NAME ${LIB_PREFIX}protobuf)

target_link_libraries(graph_proto PUBLIC ${Protobuf_LIBRARY})

install(TARGETS graph_proto
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION})
