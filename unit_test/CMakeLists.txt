include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        main
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

macro(create_test target)
  target_link_libraries(${target} gmock_main)
  add_test (NAME ${target} COMMAND $<TARGET_FILE:${target}>)
endmacro(create_test)

add_subdirectory(general)
add_subdirectory(data)
add_subdirectory(stuff)
add_subdirectory(sclam2d)
add_subdirectory(slam2d)
add_subdirectory(slam2d_addons)
add_subdirectory(slam3d)
add_subdirectory(slam3d_addons)
add_subdirectory(sim3)
add_subdirectory(sba)
add_subdirectory(solver)
