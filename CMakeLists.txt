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

include_directories(include)
# Add source files
add_executable("${TARGET_NAME}"
    src/main.cpp
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
