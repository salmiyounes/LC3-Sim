cmake_minimum_required(VERSION 3.28.3)

project(lc3)

set(SOURCES
    src/instructions.c
    src/main.c 
    src/vm.c
    src/loader.c
)

add_executable(lc3 ${SOURCES})

target_include_directories(lc3
    PRIVATE 
    ${PROJECT_SOURCE_DIR}/include
)
