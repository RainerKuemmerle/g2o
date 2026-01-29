add_executable(cylinder_fit
data.h
cyRepresentation.h
cylinder_fit.cpp
)

set_target_properties(cylinder_fit PROPERTIES OUTPUT_NAME cylinder_fit)
target_link_libraries(cylinder_fit core types_slam3d solver_dense types_sba)
