CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
SET(CMAKE_LEGACY_CYGWIN_WIN32 0)

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
SET(g2o_RUNTIME_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin CACHE PATH "Target for the binaries")
IF(WIN32)
  SET(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin CACHE PATH "Target for the libraries")
ELSE(WIN32)
  SET(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
ENDIF(WIN32)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${g2o_RUNTIME_OUTPUT_DIRECTORY})

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
LIST(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
IF(WIN32)
  ADD_DEFINITIONS(-DWINDOWS)
  MESSAGE(STATUS "Compiling on Windows")
ELSEIF(CYGWIN)
  MESSAGE(STATUS "Compiling on Cygwin")
  ADD_DEFINITIONS(-DCYGWIN)
ELSEIF(APPLE)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on OSX")
ELSEIF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ENDIF(WIN32)

# detect Android Cross Compiler
# based on android-cmake which sets the variable ANDROID for us
IF(ANDROID)
  ADD_DEFINITIONS(-DANDROID)
  MESSAGE(STATUS "Cross compiling for Android")
ENDIF()

# For building the CHOLMOD / CSPARSE solvers
OPTION (G2O_USE_CHOLMOD "Build g2o with CHOLMOD support" ON)

FIND_PACKAGE(Cholmod)
FIND_PACKAGE(BLAS)
FIND_PACKAGE(LAPACK)

IF(G2O_USE_CHOLMOD AND CHOLMOD_FOUND AND BLAS_FOUND AND LAPACK_FOUND)
  MESSAGE(STATUS "Found CHOLMOD and its dependencies")
  SET(CHOLMOD_FOUND TRUE)
ELSE()
  SET(CHOLMOD_FOUND FALSE)
ENDIF()

OPTION (G2O_USE_CSPARSE "Build g2o with CSParse support" ON)

FIND_PACKAGE(CSparse)
IF (G2O_USE_CSPARSE)
  IF(CSPARSE_FOUND)
    SET(BUILD_CSPARSE OFF CACHE BOOL "Build local CSparse library")
  ELSE(CSPARSE_FOUND)
    SET(BUILD_CSPARSE ON CACHE BOOL "Build local CSparse library")
    IF(BUILD_CSPARSE)
      SET(CSPARSE_FOUND TRUE)
    ENDIF()
  ENDIF(CSPARSE_FOUND)
ELSE(G2O_USE_CSPARSE)
  SET(BUILD_CSPARSE OFF "Build local CSparse library")
ENDIF(G2O_USE_CSPARSE)

OPTION(BUILD_LGPL_SHARED_LIBS "Build LGPL Code as Shared Libraries (LGPL Code)" ON)
SET (G2O_LGPL_LIB_TYPE STATIC)
IF (BUILD_LGPL_SHARED_LIBS)
  SET (G2O_LGPL_LIB_TYPE SHARED)
ELSE()
  MESSAGE(STATUS "Building LGPL code as static library (affects license of the binary)")
ENDIF()

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
SET(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
IF(G2O_USE_OPENMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET (G2O_OPENMP 1)
    SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
    MESSAGE(STATUS "Compiling with OpenMP support")
  ENDIF(OPENMP_FOUND)
ENDIF(G2O_USE_OPENMP)

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

include(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(G2O_BUILD_LINKED_APPS "Build apps linked with the libraries (no plugin system)" OFF
  "G2O_BUILD_APPS" OFF)

# shall we build the examples
SET(G2O_BUILD_EXAMPLES ON CACHE BOOL "Build g2o examples")
IF(G2O_BUILD_EXAMPLES)
  MESSAGE(STATUS "Compiling g2o examples")
ENDIF(G2O_BUILD_EXAMPLES)

OPTION(G2O_FAST_MATH "Enable fast math operations" OFF)
OPTION(G2O_NO_IMPLICIT_OWNERSHIP_OF_OBJECTS "Disables memory management in the graph types, this requires the callers to manager the memory of edges and nodes" OFF)

# Start of SSE* autodetect code
# (borrowed from MRPT CMake scripts, BSD)
OPTION(DO_SSE_AUTODETECT "Enable autodetection of SSE* CPU sets and enable their use in optimized code" ON)
IF(NOT EXISTS "/proc/cpuinfo")
	SET(DO_SSE_AUTODETECT OFF)
ENDIF()
IF (DO_SSE_AUTODETECT)
  FILE(READ "/proc/cpuinfo" G2O_CPU_INFO)
ENDIF()

# Macro for each SSE* var: Invoke with name in uppercase:
macro(DEFINE_SSE_VAR  _setname)
	string(TOLOWER ${_setname} _set)
	IF (DO_SSE_AUTODETECT)
		# Automatic detection:
		SET(CMAKE_G2O_HAS_${_setname} 0)
		IF (${G2O_CPU_INFO} MATCHES ".*${_set}.*")
			SET(CMAKE_G2O_HAS_${_setname} 1)
		ENDIF()
	ELSE (DO_SSE_AUTODETECT)
		# Manual:
		SET("DISABLE_${_setname}" OFF CACHE BOOL "Forces compilation WITHOUT ${_setname} extensions")
		MARK_AS_ADVANCED("DISABLE_${_setname}")
		SET(CMAKE_G2O_HAS_${_setname} 0)
		IF (NOT DISABLE_${_setname})
			SET(CMAKE_G2O_HAS_${_setname} 1)
		ENDIF (NOT DISABLE_${_setname})
	ENDIF (DO_SSE_AUTODETECT)
endmacro(DEFINE_SSE_VAR)

# SSE optimizations:
DEFINE_SSE_VAR(SSE2)
DEFINE_SSE_VAR(SSE3)
DEFINE_SSE_VAR(SSE4_1)
DEFINE_SSE_VAR(SSE4_2)
DEFINE_SSE_VAR(SSE4_A)

# Add build flags for clang AND GCC
if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
  # SSE2?
  IF (CMAKE_G2O_HAS_SSE2)
    add_compile_options(-msse2)
  ENDIF()
  # SSE3?
  IF (CMAKE_G2O_HAS_SSE3)
    add_compile_options(-msse3 -mssse3)
  ENDIF()
  # SSE4*?
  IF (CMAKE_G2O_HAS_SSE4_1)
    add_compile_options(-msse4.1)
  ENDIF()
  IF (CMAKE_G2O_HAS_SSE4_2)
    add_compile_options(-msse4.2)
  ENDIF()
  IF (CMAKE_G2O_HAS_SSE4_A)
    add_compile_options(-msse4a)
  ENDIF()
endif()
# End of of SSE* autodetect code -------

# Compiler specific options for gcc
IF(CMAKE_COMPILER_IS_GNUCXX)
  OPTION (BUILD_WITH_MARCH_NATIVE "Build with \"-march native\"" ON)
  MESSAGE(STATUS "Compiling with GCC")

  # Generic settings for optimisation
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")

  IF(G2O_FAST_MATH)
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ffast-math")
  ENDIF()

  # switch off optimization for debug builds
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

  # OS X
  #IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    #SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
    #SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
  #ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # Linux
  IF(BUILD_WITH_MARCH_NATIVE AND NOT "${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm" AND "${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
  ENDIF()
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
  #SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -stdlib=libc++")
ENDIF()

IF(MSVC)
  MESSAGE(STATUS "Compiling with MSVC")

  IF (CMAKE_GENERATOR MATCHES "ARM(64)?$")
    SET(MSVC_ARM ON)
  ENDIF()

  ADD_DEFINITIONS(-DNOMINMAX)
  ADD_DEFINITIONS(-D_USE_MATH_DEFINES)

  # exception handling
  ADD_DEFINITIONS("/EHsc")

  IF (G2O_FAST_MATH)
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /fp:fast")
  ENDIF()

  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox /Oi")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Ox /Oi")

  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

  # SSE2 optimizations
  # No need to specify if building for x64 (actually, it generates an annoying warning)
  if (NOT MSVC_ARM)
    if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        ADD_DEFINITIONS("/arch:SSE2")
    endif()
  endif()

  IF (BUILD_SHARED_LIBS)
    # disable warning on missing DLL interfaces
    ADD_DEFINITIONS("/wd4251")
  ENDIF()

  # Fix issue: https://github.com/RainerKuemmerle/g2o/issues/66
  #            Link error LNK2005 due to duplicated symbols
  add_definitions("/Ob2")
  # Fix other stupid warnings:
  add_definitions(-D_CRT_SECURE_NO_WARNINGS=1)  # Avoid deprecated fprintf(), etc.
  add_definitions("/nologo")
  # TODO not sure this should be a thing
  add_definitions("/wd4244") # Conversion from number_t -> int
  add_definitions("/wd4267") # Conversion during return
  add_definitions("/wd4522") # Duplicated operator=() in Eigen headers

ENDIF(MSVC)

# C++11 support
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Find Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)

# Set up the top-level include directories
INCLUDE_DIRECTORIES(${g2o_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR})

# Generate config.h
SET(G2O_OPENGL_FOUND ${OPENGL_FOUND})
SET(G2O_HAVE_CHOLMOD ${CHOLMOD_FOUND})
SET(G2O_HAVE_CSPARSE ${CSPARSE_FOUND})
SET(G2O_SHARED_LIBS ${BUILD_SHARED_LIBS})
SET(G2O_LGPL_SHARED_LIBS ${BUILD_LGPL_SHARED_LIBS})
SET(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(config.h.in ${PROJECT_BINARY_DIR}/g2o/config.h)
include_directories(BEFORE ${PROJECT_BINARY_DIR})
INSTALL(FILES ${PROJECT_BINARY_DIR}/g2o/config.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o)

# Include the subdirectories
ADD_SUBDIRECTORY(EXTERNAL)
ADD_SUBDIRECTORY(g2o)
