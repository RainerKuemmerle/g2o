if(CHOLMOD_INCLUDE_DIR OR CSPARSE_INCLUDE_DIR)
  add_executable(sba_demo sba_demo.cpp)
  target_link_libraries(sba_demo core types_icp types_sba)
  set_target_properties(sba_demo PROPERTIES OUTPUT_NAME sba_demo${EXE_POSTFIX})
endif()

# Link CHOLMOD if available, CSparse otherwise
if(CHOLMOD_FOUND)
  include_directories(${CHOLMOD_INCLUDE_DIR})
  target_link_libraries(sba_demo solver_cholmod)
elseif(CSPARSE_FOUND)
  include_directories(${CSPARSE_INCLUDE_DIR})
  target_link_libraries(sba_demo solver_csparse)
endif()
