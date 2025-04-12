add_executable(g2o_cli_application_linked
  ../g2o_cli/g2o.cpp)

target_link_libraries(g2o_cli_application_linked g2o_cli_library)
set_target_properties(g2o_cli_application_linked PROPERTIES OUTPUT_NAME g2o_linked)
# this disables the dynamic loading of libraries in the standard apps
target_compile_definitions(g2o_cli_application_linked G2O_DISABLE_DYNAMIC_LOADING_OF_LIBRARIES)

# linking solvers
target_link_libraries(g2o_cli_application_linked solver_csparse solver_pcg)
if(CHOLMOD_FOUND)
  target_link_libraries(g2o_cli_application_linked solver_cholmod)
endif()
# linking types
target_link_libraries(g2o_cli_application_linked types_slam2d types_slam3d types_sba types_sclam2d)

install(TARGETS g2o_cli_application_linked
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)


if(Qt5_FOUND AND QGLVIEWER_FOUND)
  include(${QT_USE_FILE})
  include_directories(${QT_INCLUDES})

  add_executable(g2o_viewer_linked
    ../g2o_viewer/g2o_viewer.cpp
    )

  target_link_libraries(g2o_viewer_linked viewer_library)
  set_target_properties(g2o_viewer_linked PROPERTIES OUTPUT_NAME g2o_viewer_linked)
  # this disables the dynamic loading of libraries in the standard apps
  target_compile_definitions(g2o_viewer_linked G2O_DISABLE_DYNAMIC_LOADING_OF_LIBRARIES)

  # linking solvers
  target_link_libraries(g2o_viewer_linked solver_csparse solver_pcg)
  if(CHOLMOD_FOUND)
    target_link_libraries(g2o_viewer_linked solver_cholmod)
  endif()
  # linking types
  target_link_libraries(g2o_viewer_linked types_slam2d types_slam3d types_sba types_sclam2d)

  install(TARGETS g2o_viewer_linked
    RUNTIME DESTINATION ${RUNTIME_DESTINATION}
    LIBRARY DESTINATION ${LIBRARY_DESTINATION}
    ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
    INCLUDES DESTINATION ${INCLUDES_DESTINATION} ${QT_INCLUDES}
  )
endif()

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/apps/linked_binaries)
