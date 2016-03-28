# download and compile gtest library
INCLUDE(gtest.cmake)

include_directories(${GTEST_INCLUDE_DIR})

ADD_SUBDIRECTORY(slam2d)
ADD_SUBDIRECTORY(slam3d)
ADD_SUBDIRECTORY(general)
