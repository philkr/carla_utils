cmake_minimum_required(VERSION 3.0)

project(CARLAUtils)
set(PROJECT_VERSION "git")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CARLA_UTILS_MAP_DIR "carla_utils/map")
include_directories("${CARLA_UTILS_MAP_DIR}")

# CARLA map source.
file(GLOB_RECURSE CARLA_MAP_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${CARLA_UTILS_MAP_DIR}/carla/*.hpp")
file(GLOB_RECURSE CARLA_MAP_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${CARLA_UTILS_MAP_DIR}/carla/*.cpp")

# Dependencies.
add_subdirectory("${PROJECT_SOURCE_DIR}/lib/pybind11")
include_directories(${PYBIND11_INCLUDE_DIR})

add_subdirectory("${PROJECT_SOURCE_DIR}/lib/pugixml")
include_directories("${PROJECT_SOURCE_DIR}/lib/pugixml/src")

# CARLA map.
add_library(carla_map ${CARLA_MAP_HEADERS} ${CARLA_MAP_SOURCES})
target_link_libraries(carla_map pugixml)

# PyBind CARLA map.
pybind11_add_module(_carla_map ${CARLA_UTILS_MAP_DIR}/_carla_map.cpp)
target_link_libraries(_carla_map PRIVATE pybind11::module carla_map)
