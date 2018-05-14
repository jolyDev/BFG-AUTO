cmake_minimum_required(VERSION 2.8.9)

set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

project (control)
file(GLOB SOURCES "src/*.cpp")

find_package(PkgConfig)

pkg_check_modules(CV REQUIRED opencv)

add_executable(${PROJECT_NAME} main.cpp ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} serial ${CV_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CV_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PUBLIC -std=c++14 -Wall ${CV_CXXFLAGS_OTHER})
