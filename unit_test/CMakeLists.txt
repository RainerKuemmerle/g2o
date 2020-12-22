# download and compile gtest library
# Download and unpack googletest at configure time
configure_file(gtest/CMakeLists.txt.in ${CMAKE_BINARY_DIR}/googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
  include_directories("${gmock_SOURCE_DIR}/include")
endif()

macro(create_test target)
  target_link_libraries(${target} gtest_main gmock)
  add_test (NAME ${target} COMMAND $<TARGET_FILE:${target}>)
endmacro(create_test)

ADD_SUBDIRECTORY(general)
ADD_SUBDIRECTORY(stuff)
ADD_SUBDIRECTORY(sclam2d)
ADD_SUBDIRECTORY(slam2d)
ADD_SUBDIRECTORY(slam2d_addons)
ADD_SUBDIRECTORY(slam3d)
ADD_SUBDIRECTORY(slam3d_addons)
ADD_SUBDIRECTORY(sim3)
ADD_SUBDIRECTORY(sba)
ADD_SUBDIRECTORY(solver)
