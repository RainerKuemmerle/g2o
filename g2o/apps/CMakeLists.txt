add_subdirectory(g2o_cli)
add_subdirectory(g2o_hierarchical)

if (G2O_HAVE_OPENGL)
  add_subdirectory(g2o_simulator)
endif()

if(QGLVIEWER_FOUND AND Qt5_FOUND)
  add_subdirectory(g2o_viewer)
endif()

if(G2O_BUILD_LINKED_APPS)
  add_subdirectory(linked_binaries)
endif()
