include_directories(${CSPARSE_INCLUDE_DIR})

add_executable(simple_optimize
  simple_optimize.cpp
)

set_target_properties(simple_optimize PROPERTIES OUTPUT_NAME simple_optimize${EXE_POSTFIX})

target_link_libraries(simple_optimize core solver_csparse)
target_link_libraries(simple_optimize types_slam2d)
target_link_libraries(simple_optimize types_slam3d)
