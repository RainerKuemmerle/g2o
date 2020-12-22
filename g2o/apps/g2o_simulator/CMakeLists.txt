add_library(g2o_simulator_library ${G2O_LIB_TYPE}
   simulator.cpp simulator.h
   pointsensorparameters.cpp  pointsensorparameters.h
   simulator2d_base.h simulator2d.h
   sensor_odometry2d.cpp sensor_odometry2d.h
   sensor_pose2d.cpp sensor_pose2d.h
   sensor_pointxy.cpp sensor_pointxy.h
   sensor_pointxy_bearing.cpp sensor_pointxy_bearing.h
   sensor_pointxy_offset.cpp sensor_pointxy_offset.h
   simutils.cpp simutils.h
   sensor_segment2d.cpp  sensor_segment2d.h
   sensor_segment2d_line.cpp  sensor_segment2d_line.h
   sensor_segment2d_pointline.cpp  sensor_segment2d_pointline.h

   simulator3d_base.h    simulator3d.h
   sensor_odometry3d.cpp sensor_odometry3d.h
   sensor_pose3d.cpp sensor_pose3d.h
   sensor_pose3d_offset.cpp sensor_pose3d_offset.h
   sensor_pointxyz.cpp sensor_pointxyz.h
   sensor_pointxyz_disparity.cpp sensor_pointxyz_disparity.h
   sensor_pointxyz_depth.cpp sensor_pointxyz_depth.h
   sensor_se3_prior.cpp sensor_se3_prior.h

   g2o_simulator_api.h
)

set_target_properties(g2o_simulator_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}simulator)
if (APPLE)
  set_target_properties(g2o_simulator_library PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

target_link_libraries(g2o_simulator_library types_slam3d_addons types_slam3d types_slam2d_addons types_slam2d  core)

add_executable(g2o_simulator2d_application
  test_simulator2d.cpp
)
target_link_libraries(g2o_simulator2d_application g2o_simulator_library types_slam2d_addons types_slam2d core)
set_target_properties(g2o_simulator2d_application PROPERTIES OUTPUT_NAME g2o_simulator2d${EXE_POSTFIX})

add_executable(convertSegmentLine_application convertSegmentLine.cpp)
target_link_libraries(convertSegmentLine_application g2o_simulator_library types_slam2d_addons types_slam3d types_slam2d core)
set_target_properties(convertSegmentLine_application PROPERTIES OUTPUT_NAME convertSegmentLine${EXE_POSTFIX})


add_executable(g2o_simulator3d_application
  test_simulator3d.cpp
)
target_link_libraries(g2o_simulator3d_application g2o_simulator_library types_slam3d_addons types_slam3d types_slam2d_addons types_slam2d core)
set_target_properties(g2o_simulator3d_application PROPERTIES OUTPUT_NAME g2o_simulator3d${EXE_POSTFIX})

add_executable(g2o_anonymize_observations_application
  g2o_anonymize_observations.cpp
)
target_link_libraries(g2o_anonymize_observations_application types_slam3d types_slam2d core)
set_target_properties(g2o_anonymize_observations_application PROPERTIES OUTPUT_NAME g2o_anonymize_observations${EXE_POSTFIX})

install(TARGETS g2o_simulator_library g2o_simulator2d_application g2o_simulator3d_application g2o_anonymize_observations_application
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/apps/g2o_simulator)
