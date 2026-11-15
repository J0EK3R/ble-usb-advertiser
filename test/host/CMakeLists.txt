# Host unit tests for the pure-C components (no ESP-IDF needed).
#   cmake -S test/host -B build-host && cmake --build build-host && ctest --test-dir build-host
cmake_minimum_required(VERSION 3.16)
project(bc2_host_tests C)
enable_testing()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(COMP ${CMAKE_CURRENT_SOURCE_DIR}/../../components)

# The two pure-C components are built with strict warnings as errors.
add_library(bc2_protocol STATIC
    ${COMP}/bc2_protocol/bc2_crc16.c
    ${COMP}/bc2_protocol/bc2_framer.c
    ${COMP}/bc2_protocol/bc2_codec.c)
target_include_directories(bc2_protocol PUBLIC ${COMP}/bc2_protocol/include)
target_compile_options(bc2_protocol PRIVATE -Wall -Wextra -Werror -Wconversion)

add_executable(bc2_tests
    test_main.c test_crc.c test_framer.c test_codec.c)
target_link_libraries(bc2_tests PRIVATE bc2_protocol)
target_compile_options(bc2_tests PRIVATE -Wall -Wextra -Werror)

add_test(NAME bc2_tests COMMAND bc2_tests)
