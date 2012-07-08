CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(g2o)

# The library prefix
SET(LIB_PREFIX g2o_)

SET(g2o_C_FLAGS)
SET(g2o_CXX_FLAGS)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# postfix, based on type
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE STRING "postfix applied to release-with-debug-information libraries")
SET(CMAKE_MINSIZEREL_POSTFIX "_s" CACHE STRING "postfix applied to minimium-size-build libraries")

# work out the postfix; required where we use OUTPUT_NAME
IF(CMAKE_BUILD_TYPE MATCHES Release)
  SET(EXE_POSTFIX)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Debug)
  SET(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  SET(EXE_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
  SET(EXE_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

# Allow the developer to select if Dynamic or Static libraries are built
OPTION (BUILD_SHARED_LIBS "Build Shared Libraries (preferred and required for the g2o plugin system)" ON)
SET (G2O_LIB_TYPE STATIC)
IF (BUILD_SHARED_LIBS)
  SET (G2O_LIB_TYPE SHARED)
ENDIF()

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# On the Mac platform, configure the RPATH as per the install, to
# avoid the problem of loading both the built and installed versions
# of the shared targets
IF(APPLE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
  SET(CMAKE_INSTALL_RPATH "")
ENDIF(APPLE)

# Set the output directory for the build executables and libraries
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin)
IF(WIN32)
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin)
ELSE(WIN32)
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/lib)
ENDIF(WIN32)

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
LIST(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
IF(WIN32)
  IF(CYGWIN)
    MESSAGE(STATUS "Compiling on Cygwin")
    ADD_DEFINITIONS(-DCYGWIN)
  ELSE(CYGWIN)
    ADD_DEFINITIONS(-DWINDOWS)
    MESSAGE(STATUS "Compiling on Windows")
  ENDIF(CYGWIN)
  IF(MSVC)
    MESSAGE(STATUS "Compiling with MSVC")
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    IF (BUILD_SHARED_LIBS)
      # disable warning on missing DLL interfaces
      ADD_DEFINITIONS("/wd4251")
      # SSE2 optimizations
      ADD_DEFINITIONS("/arch:SSE2")
    ENDIF()
  ENDIF(MSVC)
ELSEIF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ELSEIF(APPLE)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on OSX")
ENDIF(WIN32)

# For building the CHOLMOD / CSPARSE solvers
FIND_PACKAGE(SuiteSparse)
IF(CSPARSE_FOUND)
  SET(BUILD_CSPARSE OFF CACHE BOOL "Build local CSparse library")
ELSE(CSPARSE_FOUND)
  SET(BUILD_CSPARSE ON CACHE BOOL "Build local CSparse library")
  SET(CSPARSE_FOUND TRUE)
ENDIF(CSPARSE_FOUND)

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
FIND_PACKAGE(OpenMP)
SET(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
IF(OPENMP_FOUND AND G2O_USE_OPENMP)
  SET (G2O_OPENMP 1)
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Compiling with OpenMP support")
ENDIF(OPENMP_FOUND AND G2O_USE_OPENMP)

# OpenGL is used in the draw actions for the different types, as well
# as for creating the GUI itself
FIND_PACKAGE(OpenGL)
SET(G2O_USE_OPENGL ON CACHE BOOL "Build g2o with OpenGL support for visualization")
IF (OPENGL_FOUND AND G2O_USE_OPENGL)
  SET (G2O_HAVE_OPENGL 1)
  INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
  MESSAGE(STATUS "Compiling with OpenGL support")
ENDIF()

# For building the GUI
FIND_PACKAGE(QGLViewer)

# shall we build the core apps using the library
SET(G2O_BUILD_APPS ON CACHE BOOL "Build g2o apps")
IF(G2O_BUILD_APPS)
  MESSAGE(STATUS "Compiling g2o apps")
ENDIF(G2O_BUILD_APPS)

# shall we build the examples
SET(G2O_BUILD_EXAMPLES ON CACHE BOOL "Build g2o examples")
IF(G2O_BUILD_EXAMPLES)
  MESSAGE(STATUS "Compiling g2o examples")
ENDIF(G2O_BUILD_EXAMPLES)

# Compiler specific options for gcc
IF(CMAKE_COMPILER_IS_GNUCXX)
  MESSAGE(STATUS "Compiling with GCC")
  
  # Generic settings for optimisation
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse4") 
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse4") 
  # switch off optimization for debug builds
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

  # OS X
  #IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    #SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}") 
    #SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}") 
  #ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # Linux
  IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native") 
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native") 
  ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  # activate warnings !!!
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

IF(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  MESSAGE(STATUS "Compiling with Clang")

  # activate all warnings
  #SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Weverything")
  #SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Weverything")
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall")
ENDIF()


# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Find Eigen3
SET(EIGEN3_INCLUDE_DIR ${G2O_EIGEN3_INCLUDE})
FIND_PACKAGE(Eigen3)
IF(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
ELSE(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
ENDIF(EIGEN3_FOUND)

# Set up the top-level include directories
INCLUDE_DIRECTORIES(${g2o_SOURCE_DIR} ${G2O_EIGEN3_INCLUDE})

# Generate config.h
SET(G2O_OPENGL_FOUND ${OPENGL_FOUND})
SET(G2O_HAVE_CHOLMOD ${CHOLMOD_FOUND})
SET(G2O_HAVE_CSPARSE ${CSPARSE_FOUND})
SET(G2O_SHARED_LIBS ${BUILD_SHARED_LIBS})
SET(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(config.h.in ${PROJECT_BINARY_DIR}/g2o/config.h)
include_directories(${PROJECT_BINARY_DIR})
INSTALL(FILES ${PROJECT_BINARY_DIR}/g2o/config.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o)

# Include the subdirectories
ADD_SUBDIRECTORY(EXTERNAL)
ADD_SUBDIRECTORY(g2o)
