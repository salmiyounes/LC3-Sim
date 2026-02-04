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

add_executable(lc3-dbg
    src/instructions.c
    src/vm.c
    src/loader.c
    src/debugger.c
)

target_include_directories(lc3-dbg
    PRIVATE 
    ${PROJECT_SOURCE_DIR}/include
)

target_compile_definitions(lc3-dbg PRIVATE DEBUG_MODE)

enable_testing()

file(GLOB TEST_FILES "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.obj")

foreach(test_file ${TEST_FILES})
    get_filename_component(test_name ${test_file} NAME_WE)

    add_test(NAME ${test_name} COMMAND lc3 ${test_file})
    
    set_tests_properties(${test_name} PROPERTIES 
        PASS_REGULAR_EXPRESSION "VM execution halted safely"
    )
endforeach()