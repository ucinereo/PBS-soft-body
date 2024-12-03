if(TARGET igl::core)
    return()
endif()

include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/ucinereo/libigl.git
    GIT_TAG v2.5.1e
)
FetchContent_MakeAvailable(libigl)