ADD_SUBDIRECTORY(data_fitting)

ADD_SUBDIRECTORY(tutorial_slam2d)

# The condition on cholmod is required here because the cholmod solver
# is explicitly used in these examples.
IF(CHOLMOD_FOUND)
  ADD_SUBDIRECTORY(target)
  ADD_SUBDIRECTORY(ba)
ENDIF(CHOLMOD_FOUND)

ADD_SUBDIRECTORY(icp)
ADD_SUBDIRECTORY(sba)
ADD_SUBDIRECTORY(bal)

IF(QT4_FOUND AND QGLVIEWER_FOUND)
  ADD_SUBDIRECTORY(slam2d)
ENDIF(QT4_FOUND AND QGLVIEWER_FOUND)

ADD_SUBDIRECTORY(calibration_odom_laser)
ADD_SUBDIRECTORY(data_convert)
ADD_SUBDIRECTORY(interactive_slam)
