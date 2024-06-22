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

set(UNITTEST_BASE_DIR "${CMAKE_CURRENT_LIST_DIR}")
function(create_test target)
  target_sources(${target} PRIVATE ${UNITTEST_BASE_DIR}/test_helper/test_main.cpp)
  target_link_libraries(${target} gtest gmock)
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
