add_subdirectory(pcg)
add_subdirectory(dense)
add_subdirectory(slam2d_linear)
add_subdirectory(structure_only)

if(CSPARSE_FOUND)
  add_subdirectory(csparse)
endif()

if(CHOLMOD_FOUND)
  add_subdirectory(cholmod)
endif(CHOLMOD_FOUND)

# Sparse Module of Eigen is stable starting from 3.1
add_subdirectory(eigen)
