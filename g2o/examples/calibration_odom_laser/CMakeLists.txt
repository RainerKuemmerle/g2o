INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY(calibration_odom_laser_library ${G2O_LIB_TYPE}
  gm2dl_io.cpp gm2dl_io.h
  sclam_helpers.cpp sclam_helpers.h
  g2o_calibration_odom_laser_api.h
)
SET_TARGET_PROPERTIES(calibration_odom_laser_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}calibration_odom_laser)
TARGET_LINK_LIBRARIES(calibration_odom_laser_library core solver_csparse types_sclam2d types_data)

ADD_EXECUTABLE(sclam_odom_laser
  sclam_odom_laser.cpp
)
SET_TARGET_PROPERTIES(sclam_odom_laser PROPERTIES OUTPUT_NAME sclam_odom_laser${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(sclam_odom_laser calibration_odom_laser_library)

ADD_EXECUTABLE(sclam_pure_calibration
  sclam_pure_calibration.cpp
)
SET_TARGET_PROPERTIES(sclam_pure_calibration PROPERTIES OUTPUT_NAME sclam_pure_calibration${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(sclam_pure_calibration calibration_odom_laser_library)
