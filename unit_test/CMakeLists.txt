if (MSVC)
 set(BUILD_SHARED_LIBS OFF)
endif()

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        main
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# Prevent installing gtest/gmock
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
if (BUILD_SHARED_LIBS)
  target_compile_definitions(gtest PRIVATE "GTEST_CREATE_SHARED_LIBRARY=1")
  target_compile_definitions(gmock PRIVATE "GTEST_CREATE_SHARED_LIBRARY=1")
endif()

set(UNITTEST_BASE_DIR "${CMAKE_CURRENT_LIST_DIR}")
function(create_test target)
  target_sources(${target} PRIVATE ${UNITTEST_BASE_DIR}/test_helper/test_main.cpp)
  if (BUILD_SHARED_LIBS)
    set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "GTEST_LINKED_AS_SHARED_LIBRARY=1")
  endif()
  target_link_libraries(${target} gmock gtest)
  add_test (NAME ${target} COMMAND $<TARGET_FILE:${target}>)
endfunction()

add_subdirectory(test_helper)
add_subdirectory(general)
add_subdirectory(data)
add_subdirectory(stuff)
add_subdirectory(sclam2d)
add_subdirectory(slam2d)
add_subdirectory(slam3d)
add_subdirectory(sba)
add_subdirectory(sim3)
add_subdirectory(solver)

if (TARGET simulator_lib)
  add_subdirectory(simulator)
endif()
