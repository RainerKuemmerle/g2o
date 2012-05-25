ADD_EXECUTABLE(create_sphere
  create_sphere.cpp
)

SET_TARGET_PROPERTIES(create_sphere PROPERTIES OUTPUT_NAME create_sphere${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(create_sphere core types_slam3d)
