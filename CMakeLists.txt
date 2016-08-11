cmake_minimum_required(VERSION 2.8)
project(endmsf)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11 -fPIC")
SET(CMAE_BUILD_TYPE Debug)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Boost 1.54.0 REQUIRED COMPONENTS system filesystem)


INCLUDE_DIRECTORIES(
${PROJECT_SOURCE_DIR} 
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Boost_INCLUDE_DIR}
)
link_directories(${Boost_LIBRARY_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} SHARED
src/measurement.cpp
src/position_sensor.cpp
src/SSF_Core.cpp
src/state.cpp
)
target_link_libraries(
${PROJECT_NAME}
${EIGEN3_LIBS}
${Boost_LIBS}
)
add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME})






