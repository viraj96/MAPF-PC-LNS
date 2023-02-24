cmake_minimum_required(VERSION 3.16)
project(mapf_pc_lns)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -g)

find_package(plog)
find_package(BOOST 1.49.0 REQUIRED COMPONENTS program_options system filesystem)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
    ${BOOST_INCLUDE_DIRS})

file(GLOB SOURCES "src/*.cpp")

add_executable(mapf_pc_lns ${SOURCES})
target_link_libraries(mapf_pc_lns ${BOOST_LIBRARIES} plog::plog)

