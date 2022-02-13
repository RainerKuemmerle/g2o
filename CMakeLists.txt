cmake_minimum_required(VERSION 3.14)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)

project(g2o)

include(CPack)
include(GNUInstallDirs)

# The library prefix
set(LIB_PREFIX g2o_)

set(g2o_C_FLAGS)
set(g2o_CXX_FLAGS)

# default built type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# postfix, based on type
set(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")
set(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE STRING "postfix applied to release-with-debug-information libraries")
set(CMAKE_MINSIZEREL_POSTFIX "_s" CACHE STRING "postfix applied to minimium-size-build libraries")

# work out the postfix; required where we use OUTPUT_NAME
if(CMAKE_BUILD_TYPE MATCHES Release)
  set(EXE_POSTFIX)
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
  set(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
elseif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  set(EXE_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
elseif(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
  set(EXE_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
endif(CMAKE_BUILD_TYPE MATCHES Release)

# Allow the developer to select if Dynamic or Static libraries are built
option (BUILD_SHARED_LIBS "Build Shared Libraries (preferred and required for the g2o plugin system)" ON)
set (G2O_LIB_TYPE STATIC)
if (BUILD_SHARED_LIBS)
  set (G2O_LIB_TYPE SHARED)
endif()

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# On the Mac platform, configure the RPATH as per the INSTALL, to
# avoid the problem of loading both the built and INSTALLed versions
# of the shared targets
if(APPLE)
  set(CMAKE_INSTALL_RPATH "")
  set(CMAKE_MACOSX_RPATH TRUE)
  # ignore deprecated GL
  add_definitions(-DGL_SILENCE_DEPRECATION)
endif(APPLE)

# Set the output directory for the build executables and libraries
set(g2o_RUNTIME_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin CACHE PATH "Target for the binaries")
if(WIN32)
  set(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin CACHE PATH "Target for the libraries")
else(WIN32)
  set(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
endif(WIN32)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${g2o_RUNTIME_OUTPUT_DIRECTORY})

# Set standard installation directories
set(RUNTIME_DESTINATION ${CMAKE_INSTALL_BINDIR})
set(LIBRARY_DESTINATION ${CMAKE_INSTALL_LIBDIR})
set(ARCHIVE_DESTINATION ${CMAKE_INSTALL_LIBDIR})
set(INCLUDES_DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
set(INCLUDES_INSTALL_DIR ${INCLUDES_DESTINATION}/g2o)

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
list(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
if(WIN32)
  add_definitions(-DWINDOWS)
  message(STATUS "Compiling on Windows")
elseif(CYGWIN)
  message(STATUS "Compiling on Cygwin")
  add_definitions(-DCYGWIN)
elseif(APPLE)
  add_definitions(-DUNIX)
  message(STATUS "Compiling on OSX")
elseif(UNIX)
  add_definitions(-DUNIX)
  message(STATUS "Compiling on Unix")
endif(WIN32)

# detect Android Cross Compiler
# based on android-cmake which sets the variable ANDROID for us
if(ANDROID)
  add_definitions(-DANDROID)
  message(STATUS "Cross compiling for Android")
endif()

# For building the CHOLMOD based solvers
option(G2O_USE_CHOLMOD "Build g2o with CHOLMOD support" ON)
find_package(Cholmod)
find_package(BLAS)
find_package(LAPACK)
if (G2O_USE_CHOLMOD AND CHOLMOD_FOUND AND BLAS_FOUND AND LAPACK_FOUND)
  message(STATUS "Enable support for Cholmod")
  set(CHOLMOD_FOUND TRUE)
else()
  message(STATUS "Disable support for Cholmod")
  set(CHOLMOD_FOUND FALSE)
endif()

# Options to control the LGPL libraries
option(G2O_USE_LGPL_LIBS "Build libraries which use LGPL code" TRUE)

# If the LGPL libraries are used, check if static or shared libraries are used and
# show a suitable message
if (G2O_USE_LGPL_LIBS)
  if (G2O_LIB_TYPE STREQUAL "STATIC")
    message(STATUS "Building LGPL code as a static library (affects license of the binary)")
  else()
    message(STATUS "Building LGPL code as a shared library")
  endif()
endif()

# Adapter for the legacy LGPL flags. Note there is an inconsistency with the old implementation.
if (BUILD_LGPL_SHARED_LIBS)
  if (G2O_LIB_TYPE STREQUAL "SHARED")
    set(G2O_USE_LGPL_LIBS TRUE)
  else()
    message(FATAL_ERROR "BUILD_LGPL_SHARED_LIBS is set to true but G2O_LIB_TYPE is set to STATIC")
  endif()
endif()

# For building the CSparse based solvers. Note this depends on an LGPL library.
option(G2O_USE_CSPARSE "Build g2o with CSParse support" ON)
find_package(CSparse)
if (${G2O_USE_CSPARSE} AND ${CSPARSE_FOUND} AND ${G2O_USE_LGPL_LIBS})
  message(STATUS "Enable support for CSparse")
else()
  message(STATUS "Disable support for CSparse")
  set(G2O_USE_CSPARSE FALSE)
endif()


# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
set(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
if(G2O_USE_OPENMP)
  find_package(OpenMP)
  if(OPENMP_FOUND)
    set (G2O_OPENMP 1)
    set(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
    message(STATUS "Compiling with OpenMP support")
  endif(OPENMP_FOUND)
endif(G2O_USE_OPENMP)

# OpenGL is used in the draw actions for the different types, as well
# as for creating the GUI itself
set(OpenGL_GL_PREFERENCE "GLVND")
find_package(OpenGL)

# If OpenGL was found, use the import target if available. If not, use old-style includes
set(G2O_USE_OPENGL ON CACHE BOOL "Build g2o with OpenGL support for visualization")
if (OPENGL_FOUND AND G2O_USE_OPENGL)
  if (TARGET OpenGL::GL)
    set(G2O_OPENGL_TARGET "OpenGL::GL;OpenGL::GLU")
  else()
    set(G2O_OPENGL_TARGET "${OPENGL_LIBRARIES}")
    include_directories(${OPENGL_INCLUDE_DIR})
  endif()
  set (G2O_HAVE_OPENGL 1)
  message(STATUS "Compiling with OpenGL support")
  #message(WARNING G2O_OPENGL_TARGET=${G2O_OPENGL_TARGET})
endif()

# For building the GUI
find_package(QGLViewer)

# The Ceres package is required for some of the solvers.
# Ceres is vendored in g2o/EXTERNAL/ceres, or the system's can be used.
set(G2O_USE_VENDORED_CERES ON CACHE BOOL "Use vendored Ceres from g2o/EXTERNAL/ceres")

# Handle building the built in types. There is a fair degree of
# granularity and dependency in the types supported.

# 2D types
set(G2O_BUILD_SLAM2D_TYPES ON CACHE BOOL "Build SLAM2D types")
set(G2O_BUILD_SLAM2D_ADDON_TYPES ON CACHE BOOL "Build SLAM2D addon types")
set(G2O_BUILD_DATA_TYPES ON CACHE BOOL "Build SLAM2D data types")
set(G2O_BUILD_SCLAM2D_TYPES ON CACHE BOOL "Build SCLAM2D types")

# Process the arguments. If G2O_BUILD_SLAM2D_TYPES is disabled, forceably
# disable all types. Otherwise, update messages on the types enabled.
if(G2O_BUILD_SLAM2D_TYPES)
  string(APPEND supported_types_message " slam2d")
  # Make sure that we can't activate both types
  if(G2O_BUILD_SLAM2D_ADDON_TYPES)
    string(APPEND supported_types_message " slam2d (addons)")
  endif()
  if(G2O_BUILD_DATA_TYPES)
    string(APPEND supported_types_message " data")
  endif()
  if(G2O_BUILD_SCLAM2D_TYPES)
    string(APPEND supported_types_message " sclam2d")
  endif()
else()
  set(G2O_BUILD_SLAM2D_ADDON_TYPES OFF)
  set(G2O_BUILD_DATA_TYPES OFF)
  set(G2O_BUILD_SCLAM2D_TYPES OFF)
endif()

# 3D types
set(G2O_BUILD_SLAM3D_TYPES ON CACHE BOOL "Build SLAM 3D types")
set(G2O_BUILD_SLAM3D_ADDON_TYPES ON CACHE BOOL "Build SLAM 3D addon types")
set(G2O_BUILD_SBA_TYPES ON CACHE BOOL "Build SLAM3D SBA types")
set(G2O_BUILD_ICP_TYPES ON CACHE BOOL "Build SLAM3D ICP types")
set(G2O_BUILD_SIM3_TYPES ON CACHE BOOL "Build SLAM3D sim3 types")

# Process the arguments. If G2O_BUILD_SLAM3D_TYPES is disabled, forceably
# disable all the types. Enable SBA if it's not enabled but one of it's
# dependents is enabled.
if(G2O_BUILD_SLAM3D_TYPES)
  string(APPEND supported_types_message " slam3d")
  message(STATUS "Compiling SLAM 3D types")
  if (G2O_BUILD_SLAM3D_ADDON_TYPES)
    string(APPEND supported_types_message " slam3d (addons)")
  endif()
  if (G2O_BUILD_SBA_TYPES)
    string(APPEND supported_types_message " sba")
  endif()
  if (G2O_BUILD_ICP_TYPES)
    if(NOT G2O_BUILD_SBA_TYPES)
      message(WARNING "ICP types requested but SBA types not enabled; enabling SBA")
      set(G2O_BUILD_SBA_TYPES ON)
      string(APPEND supported_types_message " sba")
    endif()
    string(APPEND supported_types_message " icp")
  endif()
  if (G2O_BUILD_SIM3_TYPES)
    if(NOT G2O_BUILD_SBA_TYPES)
      message(WARNING "SIM3 types requested but SBA types not enabled; enabling SBA")
      set(G2O_BUILD_SBA_TYPES ON)
      string(APPEND supported_types_message " sba")
    endif()
    string(APPEND supported_types_message " sim3")
  endif()
else()
  set(G2O_BUILD_SLAM3D_ADDON_TYPES OFF)
  set(G2O_BUILD_SBA_TYPES OFF)
  set(G2O_BUILD_ICP_TYPES OFF)
  set(G2O_BUILD_SIM3_TYPES OFF)
endif()

if(DEFINED supported_types_message)
  message(STATUS "Compiling built in types" ${supported_types_message})
else()
  message(STATUS "Compiling with no built in types enabled")
endif()

# shall we build the core apps using the library
set(G2O_BUILD_APPS ON CACHE BOOL "Build g2o apps")
if(G2O_BUILD_APPS)
  message(STATUS "Compiling g2o apps")
endif(G2O_BUILD_APPS)

include(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(G2O_BUILD_LINKED_APPS "Build apps linked with the libraries (no plugin system)" OFF
  "G2O_BUILD_APPS" OFF)

# shall we build the examples
set(G2O_BUILD_EXAMPLES ON CACHE BOOL "Build g2o examples")
if(G2O_BUILD_EXAMPLES)
  message(STATUS "Compiling g2o examples")
endif(G2O_BUILD_EXAMPLES)

option(G2O_FAST_MATH "Enable fast math operations" OFF)
option(G2O_NO_IMPLICIT_OWNERSHIP_OF_OBJECTS "Disables memory management in the graph types, this requires the callers to manager the memory of edges and nodes" OFF)

# Start of SSE* autodetect code
# (borrowed from MRPT CMake scripts, BSD)
option(DO_SSE_AUTODETECT "Enable autodetection of SSE* CPU sets and enable their use in optimized code" ON)
if(NOT EXISTS "/proc/cpuinfo")
	set(DO_SSE_AUTODETECT OFF)
endif()
if (DO_SSE_AUTODETECT)
  file(READ "/proc/cpuinfo" G2O_CPU_INFO)
endif()

# Macro for each SSE* var: Invoke with name in uppercase:
macro(DEFINE_SSE_VAR  _setname)
	string(TOLOWER ${_setname} _set)
	if (DO_SSE_AUTODETECT)
		# Automatic detection:
		set(CMAKE_G2O_HAS_${_setname} 0)
		if (${G2O_CPU_INFO} MATCHES ".*${_set}.*")
			set(CMAKE_G2O_HAS_${_setname} 1)
		endif()
	else (DO_SSE_AUTODETECT)
		# Manual:
		set("DISABLE_${_setname}" OFF CACHE BOOL "Forces compilation WITHOUT ${_setname} extensions")
		mark_as_advanced("DISABLE_${_setname}")
		set(CMAKE_G2O_HAS_${_setname} 0)
		if (NOT DISABLE_${_setname})
			set(CMAKE_G2O_HAS_${_setname} 1)
		endif (NOT DISABLE_${_setname})
	endif (DO_SSE_AUTODETECT)
endmacro(DEFINE_SSE_VAR)

# SSE optimizations:
DEFINE_SSE_VAR(SSE2)
DEFINE_SSE_VAR(SSE3)
DEFINE_SSE_VAR(SSE4_1)
DEFINE_SSE_VAR(SSE4_2)
DEFINE_SSE_VAR(SSE4_A)

include(CheckCXXCompilerFlag)

macro(CHECK_AND_ADD_COMPILE_OPTION _option)
check_cxx_compiler_flag(${_option} OPTION_AVAILABLE)
  IF(${OPTION_AVAILABLE})
    add_compile_options(${_option})
  ENDIF(${OPTION_AVAILABLE})
endmacro(CHECK_AND_ADD_COMPILE_OPTION)

# Add build flags for clang AND GCC
if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
  # SSE2?
  if (CMAKE_G2O_HAS_SSE2)
    CHECK_AND_ADD_COMPILE_OPTION(-msse2)
  endif()
  # SSE3?
  if (CMAKE_G2O_HAS_SSE3)
    CHECK_AND_ADD_COMPILE_OPTION(-msse3 -mssse3)
  endif()
  # SSE4*?
  if (CMAKE_G2O_HAS_SSE4_1)
    CHECK_AND_ADD_COMPILE_OPTION(-msse4.1)
  endif()
  if (CMAKE_G2O_HAS_SSE4_2)
    CHECK_AND_ADD_COMPILE_OPTION(-msse4.2)
  endif()
  if (CMAKE_G2O_HAS_SSE4_A)
    CHECK_AND_ADD_COMPILE_OPTION(-msse4a)
  endif()
endif()
# End of of SSE* autodetect code -------

# code coverage
option(BUILD_CODE_COVERAGE "Enable coverage reporting" OFF)
if(BUILD_CODE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  message(STATUS "Enabling coverage compiler flags")
  set(g2o_C_FLAGS "${g2o_C_FLAGS} --coverage")
  set(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} --coverage")
endif()

# Compiler specific options for gcc
if(CMAKE_COMPILER_IS_GNUCXX)
  option (BUILD_WITH_MARCH_NATIVE "Build with \"-march native\"" OFF)
  message(STATUS "Compiling with GCC")

  # Generic settings for optimisation
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")

  if(G2O_FAST_MATH)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ffast-math")
  endif()

  # switch off optimization for debug builds
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

  # OS X
  #if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    #set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
    #set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
  #endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # Linux
  if(BUILD_WITH_MARCH_NATIVE AND NOT "${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm" AND "${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
  endif()
  # activate warnings !!!
  set(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
  set(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")
endif(CMAKE_COMPILER_IS_GNUCXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(STATUS "Compiling with Clang")

  # OS X
  if(BUILD_WITH_MARCH_NATIVE AND "${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
  endif()

  # activate all warnings
  #set(g2o_C_FLAGS "${g2o_C_FLAGS} -Weverything")
  #set(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Weverything")
  set(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall")
  set(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall")
  #set(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -stdlib=libc++")
endif()

if(MSVC)
  message(STATUS "Compiling with MSVC")

  if (CMAKE_GENERATOR MATCHES "ARM(64)?$")
    set(MSVC_ARM ON)
  endif()

  add_definitions(-DNOMINMAX)
  add_definitions(-D_USE_MATH_DEFINES)

  # exception handling
  add_definitions("/EHsc")

  if (G2O_FAST_MATH)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /fp:fast")
  endif()

  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox /Oi")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Ox /Oi")

  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /Od")

  # SSE2 optimizations
  # No need to specify if building for x64 (actually, it generates an annoying warning)
  if (NOT MSVC_ARM)
    if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        add_definitions("/arch:SSE2")
    endif()
  endif()

  if (BUILD_SHARED_LIBS)
    # disable warning on missing DLL interfaces
    add_definitions("/wd4251")
  endif()

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

  # Bessel  include(CheckIfUnderscorePrefixedBesselFunctionsExist)
  include(CheckIfUnderscorePrefixedBesselFunctionsExist)
  check_if_underscore_prefixed_bessel_functions_exist(HAVE_UNDERSCORE_PREFIXED_BESSEL_FUNCTIONS)
  if (HAVE_UNDERSCORE_PREFIXED_BESSEL_FUNCTIONS)
    add_definitions(-DCERES_MSVC_USE_UNDERSCORE_PREFIXED_BESSEL_FUNCTIONS)
  endif()

endif(MSVC)

# C++14 support
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# specifying compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Find Eigen3. If it defines the target, this is used. If not,
# fall back to the using the module form.
# See https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html for details
find_package(Eigen3 3.3 REQUIRED)
if (TARGET Eigen3::Eigen)
  set(G2O_EIGEN3_EIGEN_TARGET Eigen3::Eigen)
else()
  include_directories(${EIGEN3_INCLUDE_DIR})
endif ()

# Set up the top-level include directories
include_directories(${g2o_SOURCE_DIR} ${PROJECT_BINARY_DIR})

# Generate config.h
set(G2O_OPENGL_FOUND ${OPENGL_FOUND})
set(G2O_HAVE_CHOLMOD ${CHOLMOD_FOUND})
set(G2O_HAVE_CSPARSE ${G2O_USE_CSPARSE})
set(G2O_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(G2O_LGPL_SHARED_LIBS ${BUILD_LGPL_SHARED_LIBS})
set(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")

# Generate cmake configuration scripts
set(G2O_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(G2O_VERSION_CONFIG "${G2O_GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(G2O_PROJECT_CONFIG "${G2O_GENERATED_DIR}/${PROJECT_NAME}Config.cmake")
set(G2O_TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(G2O_CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}")
set(G2O_NAMESPACE "${PROJECT_NAME}::")
set(G2O_SRC_DIR "${PROJECT_SOURCE_DIR}")
set(G2O_VERSION 1.0.0)

include(CMakePackageConfigHelpers)
WRITE_BASIC_PACKAGE_VERSION_FILE(
    "${G2O_VERSION_CONFIG}" VERSION ${G2O_VERSION} COMPATIBILITY SameMajorVersion
)

configure_file(config.h.in "${PROJECT_BINARY_DIR}/g2o/config.h")
install(FILES ${PROJECT_BINARY_DIR}/g2o/config.h DESTINATION ${INCLUDES_DESTINATION}/g2o)

configure_file("${g2o_SOURCE_DIR}/cmake_modules/Config.cmake.in" "${G2O_PROJECT_CONFIG}" @ONLY)
configure_file("${g2o_SOURCE_DIR}/script/codecov.sh.in" "${CMAKE_CURRENT_BINARY_DIR}/codecov.sh" @ONLY)

install(
    FILES "${G2O_PROJECT_CONFIG}" "${G2O_VERSION_CONFIG}"
    DESTINATION "${G2O_CONFIG_INSTALL_DIR}")

install(
    EXPORT "${G2O_TARGETS_EXPORT_NAME}"
    NAMESPACE "${G2O_NAMESPACE}"
    DESTINATION "${G2O_CONFIG_INSTALL_DIR}")

# building unit test framework and our tests
option(BUILD_UNITTESTS "build unit test framework and the tests" OFF)
if(BUILD_UNITTESTS)
  enable_testing()
  add_subdirectory(unit_test)
endif()

# Benchmarks
option(G2O_BUILD_BENCHMARKS "build benchmarks" OFF)
if(G2O_BUILD_BENCHMARKS)
  find_package(benchmark)
  if(${benchmark_FOUND})
    add_subdirectory(benchmarks)
  else()
    message(WARNING G2O_BUILD_BENCHMARKS was set to true, but the benchmark library cannot be found)
  endif()
endif()

# Include the subdirectories
add_subdirectory(g2o)
