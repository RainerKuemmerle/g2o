add_subdirectory(g2o_cli)
add_subdirectory(g2o_hierarchical)

if (G2O_USE_OPENGL)
  add_subdirectory(g2o_simulator)
endif()

if(G2O_BUILD_VIEWER)
  add_subdirectory(g2o_viewer)
endif()

if(G2O_BUILD_LINKED_APPS)
  add_subdirectory(linked_binaries)
endif()
