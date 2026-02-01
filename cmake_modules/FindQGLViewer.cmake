# Find QGLViewer: a C++ OpenGL widget for Qt
# This module defines the following variables:
#  QGLVIEWER_FOUND        - True if QGLViewer library is found
#  QGLVIEWER_INCLUDE_DIR  - Include directories for QGLViewer
#  QGLVIEWER_LIBRARY      - QGLViewer library
#  QGLVIEWER_VERSION      - Version of QGLViewer if available

# Try pkg-config first (if available)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(QGLVIEWER QUIET QGLViewer)
endif()

# If pkg-config didn't find it, search manually
if(NOT QGLVIEWER_FOUND)
  # Try to find Qt5 (optional - QGLViewer may not be needed)
  find_package(Qt5 COMPONENTS Core Xml OpenGL Gui Widgets QUIET)
  if(NOT Qt5_FOUND)
    if(QGLViewer_FIND_REQUIRED)
      message(SEND_ERROR "Qt5 not found. Install it and set Qt5_DIR accordingly")
      if(WIN32)
        message(STATUS "  In Windows, Qt5_DIR should be something like C:/Qt/5.4/msvc2013_64_opengl/lib/cmake/Qt5")
      endif()
      return()
    else()
      message(STATUS "Qt5 not found. QGLViewer support will be disabled (this is optional)")
      return()
    endif()
  endif()

  # Find include directory
  find_path(QGLVIEWER_INCLUDE_DIR qglviewer.h
    HINTS ${QGLVIEWER_INCLUDE_DIRS}
    PATHS /usr/include/QGLViewer
          /opt/local/include/QGLViewer
          /usr/local/include/QGLViewer
          /sw/include/QGLViewer
          $ENV{QGLVIEWERROOT}/include
          $ENV{QGLVIEWERROOT}
    DOC "QGLViewer include directory"
  )

  # Find release library
  find_library(QGLVIEWER_LIBRARY_RELEASE
    NAMES qglviewer QGLViewer qglviewer-qt5 QGLViewer-qt5
    HINTS ${QGLVIEWER_LIBRARY_DIRS}
    PATHS /usr/lib
          /usr/local/lib
          /opt/local/lib
          /sw/lib
          $ENV{QGLVIEWERROOT}/lib
          $ENV{QGLVIEWERROOT}/lib64
          $ENV{QGLVIEWERROOT}/lib/x64
          $ENV{QGLVIEWERROOT}/lib/win32
    PATH_SUFFIXES QGLViewer QGLViewer/release x64/Release Win32/Release
    DOC "QGLViewer release library"
  )

  # Find debug library
  find_library(QGLVIEWER_LIBRARY_DEBUG
    NAMES dqglviewer dQGLViewer dqglviewer-qt5 dQGLViewer-qt5 QGLViewerd2
    HINTS ${QGLVIEWER_LIBRARY_DIRS}
    PATHS /usr/lib
          /usr/local/lib
          /opt/local/lib
          /sw/lib
          $ENV{QGLVIEWERROOT}/lib
          $ENV{QGLVIEWERROOT}/lib64
          $ENV{QGLVIEWERROOT}/lib/x64
          $ENV{QGLVIEWERROOT}/lib/win32
    PATH_SUFFIXES QGLViewer QGLViewer/debug x64/Debug Win32/Debug
    DOC "QGLViewer debug library"
  )

  # Handle debug/release library selection
  if(QGLVIEWER_LIBRARY_RELEASE)
    if(QGLVIEWER_LIBRARY_DEBUG)
      set(QGLVIEWER_LIBRARY optimized ${QGLVIEWER_LIBRARY_RELEASE} debug ${QGLVIEWER_LIBRARY_DEBUG})
    else()
      set(QGLVIEWER_LIBRARY ${QGLVIEWER_LIBRARY_RELEASE})
    endif()
  endif()

  # Try to extract version from header and convert to semantic version
  if(QGLVIEWER_INCLUDE_DIR AND EXISTS "${QGLVIEWER_INCLUDE_DIR}/config.h")
    file(STRINGS "${QGLVIEWER_INCLUDE_DIR}/config.h" QGLVIEWER_VERSION_LINE
      REGEX "^#define QGLVIEWER_VERSION.*")
    if(QGLVIEWER_VERSION_LINE)
      # Extract hex value (format: 0xMMmmPP where MM=major, mm=minor, PP=patch)
      string(REGEX MATCH "0x[0-9a-fA-F]+" QGLVIEWER_VERSION_HEX "${QGLVIEWER_VERSION_LINE}")
      if(QGLVIEWER_VERSION_HEX)
        # Remove 0x prefix and extract 6-digit hex string
        string(SUBSTRING "${QGLVIEWER_VERSION_HEX}" 2 6 QGLVIEWER_VERSION_HEX_DIGITS)

        # Extract major (bits 16-23, first 2 hex digits)
        string(SUBSTRING "${QGLVIEWER_VERSION_HEX_DIGITS}" 0 2 QGLVIEWER_VERSION_MAJOR_HEX)
        # Extract minor (bits 8-15, middle 2 hex digits)
        string(SUBSTRING "${QGLVIEWER_VERSION_HEX_DIGITS}" 2 2 QGLVIEWER_VERSION_MINOR_HEX)
        # Extract patch (bits 0-7, last 2 hex digits)
        string(SUBSTRING "${QGLVIEWER_VERSION_HEX_DIGITS}" 4 2 QGLVIEWER_VERSION_PATCH_HEX)

        # Convert hex to decimal
        math(EXPR QGLVIEWER_VERSION_MAJOR "0x${QGLVIEWER_VERSION_MAJOR_HEX}")
        math(EXPR QGLVIEWER_VERSION_MINOR "0x${QGLVIEWER_VERSION_MINOR_HEX}")
        math(EXPR QGLVIEWER_VERSION_PATCH "0x${QGLVIEWER_VERSION_PATCH_HEX}")

        # Assemble semantic version string
        set(QGLVIEWER_VERSION "${QGLVIEWER_VERSION_MAJOR}.${QGLVIEWER_VERSION_MINOR}.${QGLVIEWER_VERSION_PATCH}")
      endif()
    endif()
  endif()

  # Standard find_package handling
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(QGLViewer
    REQUIRED_VARS QGLVIEWER_INCLUDE_DIR QGLVIEWER_LIBRARY
    VERSION_VAR QGLVIEWER_VERSION)

  # Create modern CMake interface target
  if(QGLVIEWER_FOUND AND NOT TARGET QGLViewer::QGLViewer)
    add_library(QGLViewer::QGLViewer UNKNOWN IMPORTED)
    set_target_properties(QGLViewer::QGLViewer PROPERTIES
      IMPORTED_LOCATION "${QGLVIEWER_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${QGLVIEWER_INCLUDE_DIR}"
      INTERFACE_LINK_LIBRARIES "Qt5::Core;Qt5::Xml;Qt5::OpenGL;Qt5::Gui;Qt5::Widgets"
    )
    if(QGLVIEWER_LIBRARY_DEBUG)
      set_target_properties(QGLViewer::QGLViewer PROPERTIES
        IMPORTED_LOCATION_DEBUG "${QGLVIEWER_LIBRARY_DEBUG}"
      )
    endif()
  endif()

  mark_as_advanced(
    QGLVIEWER_INCLUDE_DIR
    QGLVIEWER_LIBRARY
    QGLVIEWER_LIBRARY_RELEASE
    QGLVIEWER_LIBRARY_DEBUG
    QGLVIEWER_VERSION_HEX
    QGLVIEWER_VERSION_HEX_DIGITS
    QGLVIEWER_VERSION_MAJOR_HEX
    QGLVIEWER_VERSION_MINOR_HEX
    QGLVIEWER_VERSION_PATCH_HEX
    QGLVIEWER_VERSION_MAJOR
    QGLVIEWER_VERSION_MINOR
    QGLVIEWER_VERSION_PATCH
    QGLVIEWER_VERSION_LINE
    QGLVIEWER_INCLUDE_DIRS
    QGLVIEWER_LIBRARY_DIRS
  )
endif()
