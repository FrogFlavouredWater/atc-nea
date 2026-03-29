cmake_minimum_required(VERSION 3.15)
project(atc VERSION 0.1 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(atc)

set(APP_SOURCES
    src/app/main.cpp
    src/app/engine.cpp
)

set(FRONTEND_SOURCES
    src/frontend/ui.cpp
)

set(CORE_SOURCES
    src/core/Config.cpp
    src/core/Logger.cpp
)

set(BACKEND_AIRCRAFT_FILES
    src/backend/aircraft/Aircraft.cpp
    src/backend/aircraft/Aircraft.h
    src/backend/aircraft/AircraftMotion.cpp
    src/backend/aircraft/AircraftMotion.h
    src/backend/aircraft/AircraftPerformance.h
)

set(BACKEND_NAVIGATION_FILES
    src/backend/navigation/GuidancePreview.cpp
    src/backend/navigation/GuidancePreview.h
    src/backend/navigation/airport.cpp
    src/backend/navigation/airport.h
)

set(SIM_SOURCES
    src/sim/ConflictDetector.cpp
    src/sim/ConflictDetector.h
    src/sim/ConflictResolver.cpp
    src/sim/ConflictResolver.h
    src/sim/Scheduler.cpp
    src/sim/Scheduler.h
    src/sim/Simulation.cpp
    src/sim/Simulation.h
    src/sim/SpawnService.cpp
    src/sim/SpawnService.h
    src/sim/TrajectoryPredictor.cpp
    src/sim/TrajectoryPredictor.h
)

target_sources(atc PRIVATE
    ${APP_SOURCES}
    ${FRONTEND_SOURCES}
    ${CORE_SOURCES}
    ${BACKEND_AIRCRAFT_FILES}
    ${BACKEND_NAVIGATION_FILES}
    ${SIM_SOURCES}
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
        "/Users/hugowb/Desktop/Main/Schoolwork/CS/vcpkg/installed/arm64-osx/include"
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
