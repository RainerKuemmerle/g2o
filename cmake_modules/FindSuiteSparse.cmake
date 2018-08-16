find_path(CHOLMOD_INCLUDE_DIR NAMES cholmod.h amd.h camd.h
    PATHS
    ${SUITE_SPARSE_ROOT}/include
    /usr/include/suitesparse
    /usr/include/ufsparse
    /opt/local/include/ufsparse
    /usr/local/include/ufsparse
    /sw/include/ufsparse
  )

find_library(CHOLMOD_LIBRARY NAMES cholmod
     PATHS
     ${SUITE_SPARSE_ROOT}/lib
     /usr/lib
     /usr/local/lib
     /opt/local/lib
     /sw/lib
   )

find_library(AMD_LIBRARY NAMES SHARED NAMES amd
  PATHS
  ${SUITE_SPARSE_ROOT}/lib
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  /sw/lib
  )

find_library(CAMD_LIBRARY NAMES camd
  PATHS
  ${SUITE_SPARSE_ROOT}/lib
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  /sw/lib
  )

find_library(SUITESPARSECONFIG_LIBRARY NAMES suitesparseconfig
  PATHS
  ${SUITE_SPARSE_ROOT}/lib
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  /sw/lib
  )


# Different platforms seemingly require linking against different sets of libraries
if(CYGWIN)
  find_package(PkgConfig)
  find_library(COLAMD_LIBRARY NAMES colamd
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
    )
  PKG_CHECK_MODULES(LAPACK lapack)

  set(CHOLMOD_LIBRARIES ${CHOLMOD_LIBRARY} ${AMD_LIBRARY} ${CAMD_LIBRARY} ${COLAMD_LIBRARY} ${CCOLAMD_LIBRARY} ${LAPACK_LIBRARIES})

# MacPorts build of the SparseSuite requires linking against extra libraries

elseif(APPLE)

  find_library(COLAMD_LIBRARY NAMES colamd
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
    )

  find_library(CCOLAMD_LIBRARY NAMES ccolamd
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
    )

  find_library(METIS_LIBRARY NAMES metis
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
    )

  set(CHOLMOD_LIBRARIES ${CHOLMOD_LIBRARY} ${AMD_LIBRARY} ${CAMD_LIBRARY} ${COLAMD_LIBRARY} ${CCOLAMD_LIBRARY} ${METIS_LIBRARY} "-framework Accelerate")
else(APPLE)
  set(CHOLMOD_LIBRARIES ${CHOLMOD_LIBRARY} ${AMD_LIBRARY})
endif(CYGWIN)

if(CHOLMOD_INCLUDE_DIR AND CHOLMOD_LIBRARIES)
  set(CHOLMOD_FOUND TRUE)
else(CHOLMOD_INCLUDE_DIR AND CHOLMOD_LIBRARIES)
  set(CHOLMOD_FOUND FALSE)
endif(CHOLMOD_INCLUDE_DIR AND CHOLMOD_LIBRARIES)

# Look for csparse; note the difference in the directory specifications!
find_path(CSPARSE_INCLUDE_DIR NAMES cs.h
  PATHS
  /usr/include/suitesparse
  /usr/include
  /opt/local/include
  /usr/local/include
  /sw/include
  /usr/include/ufsparse
  /opt/local/include/ufsparse
  /usr/local/include/ufsparse
  /sw/include/ufsparse
  )

find_library(CSPARSE_LIBRARY NAMES cxsparse
  PATHS
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  /sw/lib
  )

if(CSPARSE_INCLUDE_DIR AND CSPARSE_LIBRARY)
  set(CSPARSE_FOUND TRUE)
else(CSPARSE_INCLUDE_DIR AND CSPARSE_LIBRARY)
  set(CSPARSE_FOUND FALSE)
endif(CSPARSE_INCLUDE_DIR AND CSPARSE_LIBRARY)
