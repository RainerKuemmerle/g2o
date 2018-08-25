add_subdirectory(data_fitting)

add_subdirectory(sphere)

# The condition on cholmod is required here because the cholmod solver
# is explicitly used in these examples.
if(CHOLMOD_FOUND)
  add_subdirectory(target)
  add_subdirectory(ba)
  add_subdirectory(ba_anchored_inverse_depth)
endif(CHOLMOD_FOUND)

# Examples which explicitly use CSparse
if(CSPARSE_FOUND)
  add_subdirectory(tutorial_slam2d)
  add_subdirectory(icp)
  add_subdirectory(calibration_odom_laser)
  add_subdirectory(simple_optimize)
  add_subdirectory(plane_slam)
  add_subdirectory(line_slam)
endif()

if(CSPARSE_FOUND OR CHOLMOD_FOUND)
  add_subdirectory(sba)
  add_subdirectory(bal)
endif()

if(Qt5_FOUND AND QGLVIEWER_FOUND AND CSPARSE_FOUND)
  add_subdirectory(slam2d)
endif()

add_subdirectory(data_convert)
add_subdirectory(interactive_slam)
