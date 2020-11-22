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
  PATH_SUFFIXES
  suitesparse
  )

find_library(CSPARSE_LIBRARY NAMES cxsparse libcxsparse
  PATHS
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  /sw/lib
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CSparse DEFAULT_MSG
  CSPARSE_INCLUDE_DIR CSPARSE_LIBRARY)
