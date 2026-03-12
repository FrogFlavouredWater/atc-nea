cmake_minimum_required(VERSION 3.15)
project(atc VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# because harry said so (macos)
if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/raylib")
endif()


find_package(raylib REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(atc
        src/app/main.cpp
        src/app/engine.cpp
        src/frontend/ui.cpp
        src/common/constants.cpp
        src/common/utils.cpp
        src/common/logger.cpp
        src/backend/Aircraft.cpp
        src/backend/Aircraft.h
        src/backend/airport.h
        src/backend/airport.cpp
        src/backend/Simulation.cpp
        src/backend/Simulation.h
        src/app/main.cpp
)

target_include_directories(atc PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "$ENV{HOME}/Libraries/raygui/src"
)

target_link_libraries(atc PRIVATE
        raylib
        nlohmann_json::nlohmann_json
)

# Add compiler-specific flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(atc PRIVATE -Wall -Wextra)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(atc PRIVATE /W4 /Zc:__cplusplus)
endif()

# Ensure constants.json is copied whenever it changes
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/src/data/constants.json"
    "${CMAKE_CURRENT_BINARY_DIR}/src/data/constants.json"
    COPYONLY
)

# Also copy to the executable directory to be safe (for IDEs)
add_custom_command(TARGET atc POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:atc>/src/data"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/src/data/constants.json"
            "$<TARGET_FILE_DIR:atc>/src/data/constants.json"
)