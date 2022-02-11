cmake_minimum_required(VERSION 3.0.0)
project(ParticleFilter VERSION 0.1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if ( CMAKE_COMPILER_IS_GNUCC )
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4")
endif()

set(HEADERS
   ${HEADERS}
   ${CMAKE_CURRENT_SOURCE_DIR}/inc/Map.h
   ${CMAKE_CURRENT_SOURCE_DIR}/inc/Sensor.h
   ${CMAKE_CURRENT_SOURCE_DIR}/inc/Particle.h
   ${CMAKE_CURRENT_SOURCE_DIR}/inc/ParticleFilter.h
   ${CMAKE_CURRENT_SOURCE_DIR}/inc/ImageHandler.h
)


set(SOURCE
   ${SOURCE}
   ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/Map.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/Sensor.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/Particle.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/ParticleFilter.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/Robot.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/ImageHandler.cpp
)

include_directories(
   ${CMAKE_CURRENT_SOURCE_DIR}
   ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

add_executable(ParticleFilter ${HEADERS} ${SOURCE})

