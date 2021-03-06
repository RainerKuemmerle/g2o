add_subdirectory(slam_parser)

if(G2O_USE_CHOLMOD)
  add_subdirectory(g2o_interactive)
  add_subdirectory(g2o_incremental)
endif(G2O_USE_CHOLMOD)
