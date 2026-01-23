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
        src/main.cpp
        src/core/engine/engine.cpp
        src/core/render/ui.cpp
        src/constants/constants.cpp
        src/util/utils.cpp
        src/logger/logger.cpp
        src/core/aircraft/Aircraft.cpp
        src/core/aircraft/Aircraft.h
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

#runtime data available @ src/data/ relative to the executable working dir or sm idk
add_custom_command(TARGET atc POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:atc>/src/data"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/src/data/constants.json"
            "$<TARGET_FILE_DIR:atc>/src/data/constants.json"
)