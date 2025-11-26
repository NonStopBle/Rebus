
cmake_minimum_required(VERSION 3.10)
project(RebusAgentV3 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)
find_package(Threads REQUIRED)


include_directories(${CMAKE_SOURCE_DIR}/src)

file(GLOB_RECURSE REBUS_SOURCES
    "src/*.cpp"
)

add_executable(rebus_agent_v3 ${REBUS_SOURCES})

target_link_libraries(rebus_agent_v3 PRIVATE Threads::Threads sqlite3 hiredis )
