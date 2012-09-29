# this disables the dynamic loading of libraries in the standard apps
add_definitions(-DG2O_DISABLE_DYNAMIC_LOADING_OF_LIBRARIES)

ADD_EXECUTABLE(g2o_cli_application_linked
  ../g2o_cli/g2o.cpp)

TARGET_LINK_LIBRARIES(g2o_cli_application_linked g2o_cli_library)
SET_TARGET_PROPERTIES(g2o_cli_application_linked PROPERTIES OUTPUT_NAME g2o_linked${EXE_POSTFIX})

# linking solvers
TARGET_LINK_LIBRARIES(g2o_cli_application_linked solver_csparse solver_pcg)
IF(CHOLMOD_FOUND)
  TARGET_LINK_LIBRARIES(g2o_cli_application_linked solver_cholmod)
ENDIF()
# linking types
TARGET_LINK_LIBRARIES(g2o_cli_application_linked types_slam2d types_slam3d types_sba types_sclam2d)

INSTALL(TARGETS g2o_cli_application_linked
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)


IF(QT4_FOUND AND QGLVIEWER_FOUND)
  INCLUDE(${QT_USE_FILE})
  INCLUDE_DIRECTORIES(${QT_INCLUDES})

  ADD_EXECUTABLE(g2o_viewer_linked
    ../g2o_viewer/g2o_viewer.cpp
    )

  TARGET_LINK_LIBRARIES(g2o_viewer_linked viewer_library)
  SET_TARGET_PROPERTIES(g2o_viewer_linked PROPERTIES OUTPUT_NAME g2o_viewer_linked${EXE_POSTFIX})

  # linking solvers
  TARGET_LINK_LIBRARIES(g2o_viewer_linked solver_csparse solver_pcg)
  IF(CHOLMOD_FOUND)
    TARGET_LINK_LIBRARIES(g2o_viewer_linked solver_cholmod)
  ENDIF()
  # linking types
  TARGET_LINK_LIBRARIES(g2o_viewer_linked types_slam2d types_slam3d types_sba types_sclam2d)

  INSTALL(TARGETS g2o_viewer_linked
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  )
ENDIF()

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/apps/linked_binaries)
