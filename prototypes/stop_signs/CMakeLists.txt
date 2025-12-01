cmake_minimum_required(VERSION 3.14)
project(StopSignTest CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# download and include google test
include(FetchContent)
FetchContent_Declare(
    googletest 
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

# Enable
enable_testing()
add_executable(tests
 p1_test.cpp 
)

# link
target_link_libraries(
    tests 
    GTest::gtest_main
)

# automatically discover and register all tests
include(GoogleTest)
gtest_discover_tests(tests)