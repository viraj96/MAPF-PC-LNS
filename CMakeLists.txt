cmake_minimum_required(VERSION 3.16)
project(mapf_pc_lns)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -g -pthread)

find_package(plog)
find_package(Boost 1.71.0 REQUIRED COMPONENTS program_options system filesystem thread)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS})

file(GLOB SOURCES "src/*.cpp")

add_executable(mapf_pc_lns ${SOURCES})
target_link_libraries(mapf_pc_lns ${Boost_LIBRARIES} pthread plog::plog)
