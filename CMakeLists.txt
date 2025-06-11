cmake_minimum_required(VERSION 3.10)
project(NES VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Debug)

set(SOURCES
	src/main.cpp
	src/parse_rom.cpp
	src/mappers.cpp
	src/parse_args.cpp
	src/game_loop.cpp
	src/cpu.cpp
	src/graphics.cpp
	src/ppu.cpp

	include/main.hpp
	include/parse_rom.hpp
	include/mappers.hpp
	include/parse_args.hpp
	include/game_loop.hpp
	include/cpu.hpp
	include/ppu.hpp
	include/graphics.hpp
)

add_executable(madNES ${SOURCES})

target_include_directories(madNES PRIVATE include)
