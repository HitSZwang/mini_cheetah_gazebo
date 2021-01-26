include_directories("./")
include_directories("./Controller/")
file(GLOB_RECURSE sources
        "./*.cpp")

add_library(footstep_planner SHARED ${sources})
