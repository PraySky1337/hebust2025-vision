cmake_minimum_required(VERSION 3.15)

project(version)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-Wall -Wextra -Werror)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options(/W4 /WX)
endif()

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/src/include)
add_executable(
  ${PROJECT_NAME} 
  src/main.cpp
  src/core.cpp
  src/serial_port.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} yaml-cpp)


