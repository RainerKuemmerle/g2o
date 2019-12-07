# Provide a build target to statically link the
# g2o commandline application. The resulting binary
# includes all dependencies and can work as a
# standalone application.
#
# Static linking does not work with opengl or shared libs.
# Use the following cmake configuration:
#
#  -DG2O_USE_OPENGL=OFF
#  -DBUILD_LGPL_SHARED_LIBS=OFF
#  -DBUILD_SHARED_LIBS=OFF
#  -DG2O_BUILD_LINKED_APPS=ON
#
if (NOT (G2O_USE_OPENGL OR BUILD_LGPL_SHARED_LIBS OR BUILD_SHARED_LIBS))
    # disable dynamic loading of libraries in the standard apps
    add_compile_definitions(G2O_DISABLE_DYNAMIC_LOADING_OF_LIBRARIES)
    add_compile_definitions(G2O_STATIC_LINK_LIBRARIES)

    # statically linked cli executable
    add_executable(g2o_cli_application_static
            ../g2o_cli/g2o.cpp)

    include_directories(${CSPARSE_INCLUDE_DIR})

    # enable static linking of the g2o_cli application
    # (MacOS does not support static binaries)
    if (NOT APPLE)
        target_link_libraries(g2o_cli_application_static -static)
    endif ()
    target_link_libraries(g2o_cli_application_static g2o_cli_library)
    set_target_properties(g2o_cli_application_static PROPERTIES OUTPUT_NAME g2o_static${EXE_POSTFIX})

    # linking solvers
    target_link_libraries(g2o_cli_application_static
            solver_csparse
            solver_pcg
            solver_dense
            solver_slam2d_linear
            solver_structure_only
            solver_eigen)

    # linking types
    target_link_libraries(g2o_cli_application_static
            types_slam2d
            types_slam2d_addons
            types_slam3d
            types_slam3d_addons
            types_sba
            types_sclam2d
            )

    install(TARGETS g2o_cli_application_static
            RUNTIME DESTINATION ${RUNTIME_DESTINATION}
            LIBRARY DESTINATION ${LIBRARY_DESTINATION}
            ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
            INCLUDES DESTINATION ${INCLUDES_DESTINATION}
            )

    file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
    install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/apps/linked_binaries)

endif ()
