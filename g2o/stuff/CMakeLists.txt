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
if (APPLE)
  set_target_properties(stuff PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()

TARGET_LINK_LIBRARIES(stuff PUBLIC "${LIB_PREFIX}config")

INSTALL(TARGETS stuff
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# build our OpenGL helper library
IF(OPENGL_FOUND AND G2O_HAVE_OPENGL)
  ADD_LIBRARY(opengl_helper ${G2O_LIB_TYPE}
    opengl_primitives.cpp opengl_primitives.h
    )
  if (APPLE)
  set_target_properties(opengl_helper PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()
  INSTALL(TARGETS opengl_helper
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
  TARGET_LINK_LIBRARIES(opengl_helper PUBLIC "${OPENGL_LIBRARIES}")
  SET_TARGET_PROPERTIES(opengl_helper PROPERTIES OUTPUT_NAME "${LIB_PREFIX}opengl_helper")
  TARGET_LINK_LIBRARIES(opengl_helper PUBLIC "${LIB_PREFIX}config")
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux" AND NOT ANDROID)
  TARGET_LINK_LIBRARIES(stuff PUBLIC rt)
ENDIF()

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION include/g2o/stuff)
