add_subdirectory(g2o_cli)

if (TARGET simulator_lib)
  add_subdirectory(g2o_simulator)
endif()

if(G2O_HAVE_OPENGL AND Qt6_FOUND)
  add_subdirectory(g2o_viewer)
endif()

if(G2O_BUILD_LINKED_APPS AND G2O_BUILD_SBA_TYPES AND G2O_BUILD_SCLAM2D_TYPES)
  add_subdirectory(linked_binaries)
endif()
