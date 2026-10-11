cmake_minimum_required(VERSION 3.22)
project(CppWorkspace VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Source files
set(SOURCES
    src/main.cpp
    src/utils.cpp
)

add_executable(app_bin ${SOURCES})

target_include_directories(app_bin PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

# --- Copy Constructor Demo Executable ---
add_executable(copy_constructor_bin src/copy_constructor/main.cpp)
target_include_directories(copy_constructor_bin PRIVATE ${PROJECT_SOURCE_DIR}/include)

# Compiler warnings
if(MSVC)
    target_compile_options(app_bin PRIVATE /W4)
else()
    target_compile_options(app_bin PRIVATE -Wall -Wextra -Wpedantic -g)
endif()
