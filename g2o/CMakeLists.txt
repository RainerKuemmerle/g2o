add_subdirectory(EXTERNAL)
add_subdirectory(autodiff)
add_subdirectory(stuff)
add_subdirectory(core)

# Pre-canned types
add_subdirectory(types)

add_subdirectory(solvers)

if (G2O_BUILD_SLAM2D_TYPES AND G2O_BUILD_SLAM3D_TYPES)
  add_subdirectory(simulator)
endif()

if(G2O_BUILD_APPS)
  add_subdirectory(apps)
endif(G2O_BUILD_APPS)

# Examples
if(G2O_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif(G2O_BUILD_EXAMPLES)

if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/playground")
  # Playground
  option (G2O_BUILD_PLAYGROUND "Build g2o playground" ON)
  if (G2O_BUILD_PLAYGROUND)
    message(STATUS "building playground")
    add_subdirectory(playground)
  endif()
endif()
