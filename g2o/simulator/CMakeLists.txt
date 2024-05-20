add_library(simulator_lib ${G2O_LIB_TYPE}
   simulator.cpp simulator.h
   pointsensorparameters.cpp  pointsensorparameters.h
   simulator2d_base.cpp simulator2d_base.h
   sensor_odometry2d.cpp sensor_odometry2d.h
   sensor_pose2d.cpp sensor_pose2d.h
   sensor_pointxy.cpp sensor_pointxy.h
   sensor_pointxy_bearing.cpp sensor_pointxy_bearing.h
   sensor_pointxy_offset.cpp sensor_pointxy_offset.h
   simutils.cpp simutils.h
   sensor_segment2d.cpp  sensor_segment2d.h
   sensor_segment2d_line.cpp  sensor_segment2d_line.h
   sensor_segment2d_pointline.cpp  sensor_segment2d_pointline.h

   simulator3d_base.h
   sensor_odometry3d.cpp sensor_odometry3d.h
   sensor_pose3d.cpp sensor_pose3d.h
   sensor_pose3d_offset.cpp sensor_pose3d_offset.h
   sensor_pointxyz.cpp sensor_pointxyz.h
   sensor_pointxyz_disparity.cpp sensor_pointxyz_disparity.h
   sensor_pointxyz_depth.cpp sensor_pointxyz_depth.h
   sensor_se3_prior.cpp sensor_se3_prior.h

   g2o_simulator_api.h
)

target_include_directories(core PUBLIC
  "$<BUILD_INTERFACE:${g2o_SOURCE_DIR};${PROJECT_BINARY_DIR}>"
  $<INSTALL_INTERFACE:include/g2o/simulator>
)

set_target_properties(simulator_lib PROPERTIES OUTPUT_NAME ${LIB_PREFIX}simulator)
set_target_properties(simulator_lib PROPERTIES
  VERSION ${G2O_LIB_VERSION}
  SOVERSION ${G2O_LIB_SOVERSION})
if (APPLE)
  set_target_properties(simulator_lib PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

target_link_libraries(simulator_lib types_slam3d_addons types_slam3d types_slam2d_addons types_slam2d core)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/simulator)

install(TARGETS simulator_lib
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)
