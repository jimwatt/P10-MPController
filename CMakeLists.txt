project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -04 for best performance
add_definitions(-std=c++11 -O4)

set(CXX_FLAGS "-Wall -Werror -pedantic -DHAVE_CSTDDEF -I/usr/include/coin")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/MPC.cpp src/utilities.cpp src/main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)


add_executable(mpc ${sources})

target_link_libraries(mpc ipopt z ssl uv uWS)

