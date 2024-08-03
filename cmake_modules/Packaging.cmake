# RPATH setup
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif()

# helper for splitting version string
macro(version_split version_str major minor patch)
  string(REGEX REPLACE "([0-9]+).[0-9]+.[0-9]+" "\\1" ${major} ${version_str})
  string(REGEX REPLACE "[0-9]+.([0-9]+).[0-9]+" "\\1" ${minor} ${version_str})
  string(REGEX REPLACE "[0-9]+.[0-9]+.([0-9]+)" "\\1" ${patch} ${version_str})
endmacro()

version_split(${G2O_VERSION} g2o_major g2o_minor g2o_patch)

set(CPACK_PACKAGE_NAME ${PROJECT_NAME}
    CACHE STRING "g2o"
)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "g2o: General Graph Optimization" CACHE
  STRING "g2o is an open-source C++ framework for optimizing graph-based
  nonlinear error functions. g2o has been designed to be easily extensible to a
  wide range of problems and a new problem typically can be specified in a few
  lines of code. The current implementation provides solutions to several
  variants of SLAM and BA."
)

set(CPACK_VERBATIM_VARIABLES YES)

set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
SET(CPACK_OUTPUT_FILE_PREFIX "${PROJECT_BINARY_DIR}/_packages")

set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CPACK_PROJECT_CONFIG_FILE ${CMAKE_SOURCE_DIR}/cmake_modules/PackagingConfig.cmake)
set(CPACK_STRIP_FILES YES)

set(CPACK_PACKAGE_VERSION_MAJOR ${g2o_major})
set(CPACK_PACKAGE_VERSION_MINOR ${g2o_minor})
set(CPACK_PACKAGE_VERSION_PATCH ${g2o_patch})

set(CPACK_PACKAGE_CONTACT "Rainer Kuemmerle <rainer.kuemmerle@gmail.com>")

set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)
set(CPACK_DEB_COMPONENT_INSTALL YES)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS YES)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS_PRIVATE_DIRS "${g2o_LIBRARY_OUTPUT_DIRECTORY}")

include(CPack)
