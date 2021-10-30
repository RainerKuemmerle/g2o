# Self-contained examples which declare their own types
add_subdirectory(data_fitting)
add_subdirectory(dynamic_vertex)
add_subdirectory(bal)
add_subdirectory(target)
add_subdirectory(tutorial_slam2d)

# 2D SLAM examples

if(G2O_BUILD_SCLAM2D_TYPES AND G2O_BUILD_DATA_TYPES)
  add_subdirectory(calibration_odom_laser)
endif()

# 3D examples
if(G2O_BUILD_SBA_TYPES)
  add_subdirectory(ba)
  add_subdirectory(ba_anchored_inverse_depth)
  add_subdirectory(data_convert)
endif()

if (G2O_BUILD_ICP_TYPES)
  add_subdirectory(icp)
  add_subdirectory(sba)
endif()

if (G2O_BUILD_SLAM3D_ADDON_TYPES)
  add_subdirectory(line_slam)
  add_subdirectory(plane_slam)
endif()

if (G2O_BUILD_SIM3_TYPES)
  add_subdirectory(sim3)
endif()

# slam 3d
if (G2O_BUILD_SLAM3D_TYPES)
  add_subdirectory(sphere)
endif()

if(G2O_BUILD_SLAM2D_TYPES AND G2O_BUILD_SLAM3D_TYPES)
  add_subdirectory(simple_optimize)
  add_subdirectory(interactive_slam)
endif()

if(Qt5_FOUND AND QGLVIEWER_FOUND)
  add_subdirectory(slam2d)
endif()

