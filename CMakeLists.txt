cmake_minimum_required(VERSION 3.15)
project(atc VERSION 0.1 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(atc)

target_sources(atc PRIVATE
    src/app/main.cpp
    src/app/engine.cpp
    src/frontend/ui.cpp
    src/common/utils.cpp
    src/common/logger.cpp
    src/backend/Aircraft.cpp
    src/backend/airport.cpp
    src/backend/Simulation.cpp
    src/backend/Aircraft.h
    src/backend/AircraftControl.h
    src/backend/airport.h
    src/backend/Simulation.h
)

target_compile_features(atc PRIVATE cxx_std_23)

target_include_directories(atc PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

set(_raylib_hints "")
set(_raygui_hints "")

if(APPLE)
    list(APPEND _raylib_hints "/opt/homebrew/opt/raylib")
    list(APPEND _raygui_hints
        "/opt/homebrew/include"
        "/opt/homebrew/opt/raygui/include"
    )

    find_program(BREW_EXECUTABLE brew)
    if(BREW_EXECUTABLE)
        execute_process(
            COMMAND "${BREW_EXECUTABLE}" --prefix
            OUTPUT_VARIABLE _brew_prefix
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(_brew_prefix)
            list(APPEND _raylib_hints "${_brew_prefix}/opt/raylib")
            list(APPEND _raygui_hints
                "${_brew_prefix}/include"
                "${_brew_prefix}/opt/raygui/include"
            )
        endif()
    endif()
endif()

find_package(raylib REQUIRED CONFIG HINTS ${_raylib_hints})

find_path(RAYGUI_INCLUDE_DIR
    NAMES raygui.h
    HINTS ${_raygui_hints}
)

if(NOT RAYGUI_INCLUDE_DIR)
    find_path(RAYGUI_INCLUDE_DIR NAMES raygui.h)
endif()

if(NOT RAYGUI_INCLUDE_DIR)
    message(FATAL_ERROR "raygui.h not found. Install raygui or add it to your include path.")
endif()

target_include_directories(atc PRIVATE "${RAYGUI_INCLUDE_DIR}")

if(TARGET raylib::raylib)
    target_link_libraries(atc PRIVATE raylib::raylib)
elseif(TARGET raylib)
    target_link_libraries(atc PRIVATE raylib)
else()
    message(FATAL_ERROR "raylib target not found after find_package(raylib).")
endif()

if(MSVC)
    target_compile_options(atc PRIVATE /W4 /Zc:__cplusplus)
else()
    target_compile_options(atc PRIVATE -Wall -Wextra)
endif()
