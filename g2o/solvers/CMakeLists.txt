add_subdirectory(pcg)
add_subdirectory(dense)
add_subdirectory(structure_only)

if(G2O_USE_CSPARSE)
  add_subdirectory(csparse)
  add_subdirectory(slam2d_linear)
endif()

if(G2O_USE_CHOLMOD)
  add_subdirectory(cholmod)
endif(G2O_USE_CHOLMOD)

# Sparse Module of Eigen is stable starting from 3.1
if (DEFINED EIGEN3_VERSION)
  if(${EIGEN3_VERSION} VERSION_GREATER "3.1.0" OR ${EIGEN3_VERSION} VERSION_EQUAL "3.1.0")
    add_subdirectory(eigen)
  endif()
endif()
