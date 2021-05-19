add_subdirectory(pcg)
add_subdirectory(dense)

if(G2O_BUILD_SLAM2D_TYPES)
  add_subdirectory(slam2d_linear)
endif()

add_subdirectory(structure_only)

if(CSPARSE_FOUND)
  add_subdirectory(csparse)
endif()

if(CHOLMOD_FOUND)
  add_subdirectory(cholmod)
endif(CHOLMOD_FOUND)

# Sparse Module of Eigen is stable starting from 3.1
add_subdirectory(eigen)
