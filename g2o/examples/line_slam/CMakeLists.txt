INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

# ADD_EXECUTABLE(line_test line_test.cpp)
# TARGET_LINK_LIBRARIES(line_test types_slam3d_addons types_slam3d)
# SET_TARGET_PROPERTIES(line_test PROPERTIES OUTPUT_NAME line_test${EXE_POSTFIX})

ADD_EXECUTABLE(simulator_3d_line simulator_3d_line.cpp)
TARGET_LINK_LIBRARIES(simulator_3d_line solver_csparse types_slam3d_addons)
SET_TARGET_PROPERTIES(simulator_3d_line PROPERTIES OUTPUT_NAME simulator_3d_line${EXE_POSTFIX})
