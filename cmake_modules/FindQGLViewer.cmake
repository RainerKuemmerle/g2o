# Need to find both Qt4 and QGLViewer if the QQL support is to be built
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtXml QtOpenGL QtGui)

FIND_PATH(QGLVIEWER_INCLUDE_DIR qglviewer.h
    /usr/include/QGLViewer
    /opt/local/include/QGLViewer
    /usr/local/include/QGLViewer
    /sw/include/QGLViewer
  )


IF(WIN32)
  FIND_LIBRARY(QGLVIEWER_RELEASE_LIBRARY QGLViewer2)
  FIND_LIBRARY(QGLVIEWER_DEBUG_LIBRARY   QGLViewerd2)

  IF(QGLVIEWER_DEBUG_LIBRARY AND QGLVIEWER_RELEASE_LIBRARY)
    set(QGLVIEWER_LIBRARY debug  ${QGLVIEWER_DEBUG_LIBRARY}
                          optimized ${QGLVIEWER_RELEASE_LIBRARY} )
  ENDIF(QGLVIEWER_DEBUG_LIBRARY AND QGLVIEWER_RELEASE_LIBRARY)

ELSE(WIN32)
  FIND_LIBRARY(QGLVIEWER_LIBRARY NAMES  qglviewer-qt4 QGLViewer
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
    )
ENDIF(WIN32)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(QGLVIEWER DEFAULT_MSG
  QGLVIEWER_INCLUDE_DIR QGLVIEWER_LIBRARY)
