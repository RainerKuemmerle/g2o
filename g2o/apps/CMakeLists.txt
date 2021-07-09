add_subdirectory(g2o_cli)

if (G2O_BUILD_SLAM3D_TYPES)
  add_subdirectory(g2o_hierarchical)
endif()

if (G2O_HAVE_OPENGL AND G2O_BUILD_SLAM2D_TYPES  AND G2O_BUILD_SLAM3D_TYPES)
  add_subdirectory(g2o_simulator)
endif()

if(QGLVIEWER_FOUND AND Qt5_FOUND)
  add_subdirectory(g2o_viewer)
endif()

if(G2O_BUILD_LINKED_APPS AND G2O_BUILD_SBA_TYPES AND G2O_BUILD_SCLAM2D_TYPES)
  add_subdirectory(linked_binaries)
endif()
