# Pre-canned types

# 2D types
if(G2O_BUILD_SLAM2D_TYPES)
  add_subdirectory(slam2d)
  if(G2O_BUILD_SLAM2D_ADDON_TYPES)
    add_subdirectory(slam2d_addons)
  endif()
  if(G2O_BUILD_DATA_TYPES)
    add_subdirectory(data)
  endif()
  if(G2O_BUILD_SCLAM2D_TYPES)
    add_subdirectory(sclam2d)
  endif()
endif()

# 3D types
if(G2O_BUILD_SLAM3D_TYPES)
  add_subdirectory(slam3d)
  if(G2O_BUILD_SLAM3D_ADDON_TYPES)
    add_subdirectory(slam3d_addons)
  endif()
  if(G2O_BUILD_SBA_TYPES)
    add_subdirectory(sba)
    if(G2O_BUILD_ICP_TYPES)
      add_subdirectory(icp)
    endif()
    if (G2O_BUILD_SIM3_TYPES)
      add_subdirectory(sim3)
    endif()
  endif()
endif()
