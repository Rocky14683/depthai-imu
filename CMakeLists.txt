cmake_minimum_required(VERSION 3.4)

# Add depthai-core dependency
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core EXCLUDE_FROM_ALL)

# Create a project with name 'myapp'
set(TARGET_NAME depthai-driver)
project(${TARGET_NAME})

# Dependencies (optional, only used for example)
find_package(OpenCV REQUIRED)

include(FetchContent)
FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# google test
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

include_directories(include)
include_directories(include/vector_datatype)
include_directories(include/filter)
# Add source files
add_executable("${TARGET_NAME}"
    src/main.cpp
        src/vector_datatype/quaternion_type.cpp
        src/vector_datatype/vector_type.cpp
        src/filter/imuFilter.cpp
        src/filter/accIntegral.cpp
)

# Link with libraries
target_link_libraries(${TARGET_NAME}
    PUBLIC
        depthai::opencv
        ${OpenCV_LIBS} # optional, used for example
        rerun_sdk
)

# Set some errors
if(NOT MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Werror=return-type>)
endif()

# Set compiler features (c++23)
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 23)



# for google test
enable_testing()

file(GLOB TEST_SOURCES test/*.cpp)
add_executable(GTEST ${TEST_SOURCES})
target_include_directories(GTEST PRIVATE include)
# link against the standard C++ library so that coroutines work
target_link_libraries(GTEST
        PUBLIC
        gtest
        gtest_main)

include(GoogleTest)
gtest_discover_tests(GTEST)
