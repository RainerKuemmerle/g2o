# Need to find both Qt4 and QGLViewer if the QQL support is to be built
FIND_PACKAGE(Qt4 COMPONENTS QtXml QtOpenGL QtGui)

FIND_PATH(QGLVIEWER_INCLUDE_DIR qglviewer.h
    /usr/include/QGLViewer
    /opt/local/include/QGLViewer
    /usr/local/include/QGLViewer
    /sw/include/QGLViewer
  )

FIND_LIBRARY(QGLVIEWER_LIBRARY NAMES  qglviewer-qt4 QGLViewer
  PATHS
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  /sw/lib
  )


IF(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)
  SET(QGLVIEWER_FOUND TRUE)
ELSE(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)
  SET(QGLVIEWER_FOUND FALSE)
ENDIF(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)
