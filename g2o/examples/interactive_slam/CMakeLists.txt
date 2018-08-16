add_subdirectory(slam_parser)

if(CHOLMOD_FOUND)
  add_subdirectory(g2o_interactive)
  add_subdirectory(g2o_incremental)
endif(CHOLMOD_FOUND)
