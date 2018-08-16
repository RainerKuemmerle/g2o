include_directories(${CSPARSE_INCLUDE_DIR})

add_library(calibration_odom_laser_library ${G2O_LIB_TYPE}
  gm2dl_io.cpp gm2dl_io.h
  sclam_helpers.cpp sclam_helpers.h
  motion_information.h
  edge_se2_pure_calib.cpp edge_se2_pure_calib.h
  closed_form_calibration.cpp closed_form_calibration.h
  g2o_calibration_odom_laser_api.h
)
set_target_properties(calibration_odom_laser_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}calibration_odom_laser)
target_link_libraries(calibration_odom_laser_library core solver_csparse types_sclam2d types_data)

add_executable(sclam_odom_laser
  sclam_odom_laser.cpp
)
set_target_properties(sclam_odom_laser PROPERTIES OUTPUT_NAME sclam_odom_laser${EXE_POSTFIX})
target_link_libraries(sclam_odom_laser calibration_odom_laser_library)

add_executable(sclam_pure_calibration
  sclam_pure_calibration.cpp
)
set_target_properties(sclam_pure_calibration PROPERTIES OUTPUT_NAME sclam_pure_calibration${EXE_POSTFIX})
target_link_libraries(sclam_pure_calibration calibration_odom_laser_library)

add_executable(sclam_laser_calib
  sclam_laser_calib.cpp
)
set_target_properties(sclam_laser_calib PROPERTIES OUTPUT_NAME sclam_laser_calib${EXE_POSTFIX})
target_link_libraries(sclam_laser_calib calibration_odom_laser_library)
