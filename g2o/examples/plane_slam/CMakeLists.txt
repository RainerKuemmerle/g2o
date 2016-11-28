INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_EXECUTABLE(simulator_3d_plane
  simulator_3d_plane.cpp
)
TARGET_LINK_LIBRARIES(simulator_3d_plane solver_csparse types_slam3d_addons)
SET_TARGET_PROPERTIES(simulator_3d_plane PROPERTIES OUTPUT_NAME simulator_3d_plane${EXE_POSTFIX})
