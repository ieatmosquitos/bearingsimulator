# CMake for bearing only robot simulator
cmake_minimum_required(VERSION 2.8)

# Project Information
project(bearingsimulator)
set(bearingsimulator_VERSION_MAJOR 0)
set(bearingsimulator_VERSION_MINOR 1)

# Fetch Source Files
file(GLOB_RECURSE SIMSOURCES src/*.cpp)

# Add local cmake_modules folder in CMake search path
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

# Search for required libraries 
find_package(SFML 1.6 COMPONENTS system window graphics network audio REQUIRED)
find_package(EIGEN REQUIRED)

# Include all include directories
include_directories(${SFML_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories("${CMAKE_SOURCE_DIR}/include")

# Build Executable
add_executable(simulator ${SIMSOURCES})

# Link all the libraries!
target_link_libraries(simulator ${SFML_LIBRARIES} ${EIGEN_LIBS})

# Post build operation.
# Copy required assets and shader from source to build.
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/assets
    ${CMAKE_BINARY_DIR}/assets
)