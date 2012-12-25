ADD_LIBRARY(g2o_simulator_library ${G2O_LIB_TYPE}
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

SET_TARGET_PROPERTIES(g2o_simulator_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}simulator)

TARGET_LINK_LIBRARIES(g2o_simulator_library types_slam3d_addons types_slam3d types_slam2d_addons types_slam2d  core)

ADD_EXECUTABLE(g2o_simulator2d_application
  test_simulator2d.cpp
)
TARGET_LINK_LIBRARIES(g2o_simulator2d_application g2o_simulator_library types_slam2d_addons types_slam2d core)
SET_TARGET_PROPERTIES(g2o_simulator2d_application PROPERTIES OUTPUT_NAME g2o_simulator2d${EXE_POSTFIX})

ADD_EXECUTABLE(convertSegmentLine_application convertSegmentLine.cpp)

TARGET_LINK_LIBRARIES(convertSegmentLine_application g2o_simulator_library types_slam2d_addons types_slam3d types_slam2d core)
SET_TARGET_PROPERTIES(convertSegmentLine_application PROPERTIES OUTPUT_NAME convertSegmentLine${EXE_POSTFIX})


ADD_EXECUTABLE(g2o_simulator3d_application
  test_simulator3d.cpp
)
TARGET_LINK_LIBRARIES(g2o_simulator3d_application g2o_simulator_library types_slam3d_addons types_slam3d types_slam2d_addons types_slam2d core)
SET_TARGET_PROPERTIES(g2o_simulator3d_application PROPERTIES OUTPUT_NAME g2o_simulator3d${EXE_POSTFIX})

ADD_EXECUTABLE(g2o_anonymize_observations_application
  g2o_anonymize_observations.cpp
)
TARGET_LINK_LIBRARIES(g2o_anonymize_observations_application types_slam3d types_slam2d core)
SET_TARGET_PROPERTIES(g2o_anonymize_observations_application PROPERTIES OUTPUT_NAME g2o_anonymize_observations${EXE_POSTFIX})

INSTALL(TARGETS g2o_simulator_library g2o_simulator2d_application g2o_simulator3d_application g2o_anonymize_observations_application
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/apps/g2o_simulator)
