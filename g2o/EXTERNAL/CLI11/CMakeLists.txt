include(FetchContent)
FetchContent_Declare(
    cli11_proj
    QUIET
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    # v2.5.0
    GIT_TAG 4160d259d961cd393fd8d67590a8c7d210207348
)

FetchContent_MakeAvailable(cli11_proj)
