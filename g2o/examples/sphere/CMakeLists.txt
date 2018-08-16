add_executable(create_sphere
  create_sphere.cpp
)

set_target_properties(create_sphere PROPERTIES OUTPUT_NAME create_sphere${EXE_POSTFIX})
target_link_libraries(create_sphere core types_slam3d)
