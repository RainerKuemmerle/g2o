ADD_SUBDIRECTORY(data_fitting)

ADD_SUBDIRECTORY(sphere)

# The condition on cholmod is required here because the cholmod solver
# is explicitly used in these examples.
IF(CHOLMOD_FOUND)
  ADD_SUBDIRECTORY(target)
  ADD_SUBDIRECTORY(ba)
  ADD_SUBDIRECTORY(ba_anchored_inverse_depth)
ENDIF(CHOLMOD_FOUND)

# Examples which explicitly use CSparse
IF(CSPARSE_FOUND)
  ADD_SUBDIRECTORY(tutorial_slam2d)
  ADD_SUBDIRECTORY(icp)
  ADD_SUBDIRECTORY(calibration_odom_laser)
  ADD_SUBDIRECTORY(simple_optimize)
  ADD_SUBDIRECTORY(plane_slam)
ENDIF()

IF(CSPARSE_FOUND OR CHOLMOD_FOUND)
  ADD_SUBDIRECTORY(sba)
  ADD_SUBDIRECTORY(bal)
ENDIF()

IF(QT4_FOUND AND QGLVIEWER_FOUND AND CSPARSE_FOUND)
  ADD_SUBDIRECTORY(slam2d)
ENDIF()

ADD_SUBDIRECTORY(data_convert)
ADD_SUBDIRECTORY(interactive_slam)
