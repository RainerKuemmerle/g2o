ADD_LIBRARY(stuff ${G2O_LIB_TYPE}
  filesys_tools.h    string_tools.h
  color_macros.h     macros.h         timeutil.cpp
  command_args.cpp   misc.h           sparse_helper.cpp  timeutil.h
  command_args.h     os_specific.c    sparse_helper.h
  filesys_tools.cpp  os_specific.h    string_tools.cpp
  property.cpp       property.h       
  sampler.cpp        sampler.h        unscented.h
  tictoc.cpp tictoc.h
  g2o_stuff_api.h
)

SET_TARGET_PROPERTIES(stuff PROPERTIES OUTPUT_NAME ${LIB_PREFIX}stuff)

INSTALL(TARGETS stuff
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

# build our OpenGL helper library
IF(OPENGL_FOUND AND G2O_HAVE_OPENGL)
  ADD_LIBRARY(opengl_helper ${G2O_LIB_TYPE}
    opengl_primitives.cpp opengl_primitives.h
  )
  INSTALL(TARGETS opengl_helper
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  )
  TARGET_LINK_LIBRARIES(opengl_helper ${OPENGL_LIBRARIES})
  SET_TARGET_PROPERTIES(opengl_helper PROPERTIES OUTPUT_NAME ${LIB_PREFIX}opengl_helper)
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux" AND NOT ANDROID)
  TARGET_LINK_LIBRARIES(stuff rt)
ENDIF()

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/stuff)
