cmake_minimum_required(VERSION 3.10)
project(NES VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE Debug)

set(SOURCES
	src/main.cpp
	src/parse_rom.cpp
	src/mappers.cpp
	src/parse_args.cpp
	# src/apu.cpp
	# src/cpu.cpp
	# src/ppu.cpp

	include/main.hpp
	include/parse_rom.hpp
	include/mappers.hpp
	include/parse_args.hpp
	# src/apu.hpp
	# src/cpu.hpp
	# src/ppu.hpp
)

add_executable(nes ${SOURCES})

target_include_directories(nes PRIVATE include)
