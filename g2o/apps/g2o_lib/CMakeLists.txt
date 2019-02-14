# Dynamic g2o library without dependencies.
#
# Create a g2o library that mimics the g2o cli executable and
# includes all solvers, types, and kernels.
#
#  -DG2O_USE_OPENGL=OFF
#  -DBUILD_LGPL_SHARED_LIBS=OFF
#  -DBUILD_SHARED_LIBS=OFF
#  -DCMAKE_POSITION_INDENPENDENT_CODE=ON
#
if (NOT (G2O_USE_OPENGL OR BUILD_LGPL_SHARED_LIBS OR BUILD_SHARED_LIBS OR NOT CMAKE_POSITION_INDEPENDENT_CODE))
    # disable dynamic loading of libraries in the standard apps
    add_definitions(-DG2O_DISABLE_DYNAMIC_LOADING_OF_LIBRARIES)

    # g2o library without dependencies
    add_library(g2o_lib SHARED
            g2o_lib.cpp
            stream_interceptor.cpp
            stream_interceptor.h)

    include_directories(${CSPARSE_INCLUDE_DIR})

    target_link_libraries(g2o_lib g2o_cli_library)

    # linking solvers
    target_link_libraries(g2o_lib
            solver_csparse
            solver_pcg
            solver_dense
            solver_slam2d_linear
            solver_structure_only
            solver_eigen)

    # linking types
    target_link_libraries(g2o_lib
            types_slam2d
            types_slam2d_addons
            types_slam3d
            types_slam3d_addons
            types_sba
            types_sclam2d
            )

    install(TARGETS g2o_lib
            RUNTIME DESTINATION ${RUNTIME_DESTINATION}
            LIBRARY DESTINATION ${LIBRARY_DESTINATION}
            ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
            INCLUDES DESTINATION ${INCLUDES_DESTINATION}
            )

    file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
    install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/apps/linked_binaries)

endif ()