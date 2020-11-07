include_directories(${CSPARSE_INCLUDE_DIR})

add_executable(dynamic_vertex dynamic_vertex.cpp)
set_target_properties(dynamic_vertex PROPERTIES OUTPUT_NAME dynamic_vertex${EXE_POSTFIX})
target_link_libraries(dynamic_vertex core solver_csparse)


add_executable(polynomial_fit polynomial_fit.cpp)
set_target_properties(polynomial_fit PROPERTIES OUTPUT_NAME polynomial_fit${EXE_POSTFIX})
target_link_libraries(polynomial_fit core solver_csparse)
