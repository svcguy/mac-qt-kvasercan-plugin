cmake_minimum_required(VERSION 3.18)

# This plugin is only for MacOS (at the moment)
if(NOT APPLE)
    message(FATAL_ERROR "This plugin is only for MacOS (at the moment)")
endif()

# The KVaserCAN library uses Make, so we'll use ExternalProject to build it
include(ExternalProject)

# The project
project(kvasercan LANGUAGES CXX)

# Qt Stuff
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Gather up the required Qt packages
find_package(Qt6 REQUIRED COMPONENTS Core SerialBus)

# Here's where we build the KVaserCAN Library
ExternalProject_Add(kvasercan-library
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/KvaserCAN-Library
    DOWNLOAD_COMMAND ""
    CONFIGURE_COMMAND "./build_no.sh"
    BUILD_COMMAND make all
    BUILD_IN_SOURCE true
    INSTALL_COMMAND ""
)

# Our sources
add_library(kvasercan SHARED
    main.cpp
    kvasercanbackend.cpp kvasercanbackend_p.h kvasercanbackend.h
    plugin.json
)

# Add headers for KVaserCAN-Library
target_include_directories(kvasercan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/KvaserCAN-Library/Sources
    ${CMAKE_CURRENT_SOURCE_DIR}/KvaserCAN-Library/Sources/CANAPI
    ${CMAKE_CURRENT_SOURCE_DIR}/KvaserCAN-Library/Sources/Driver
    ${CMAKE_CURRENT_SOURCE_DIR}/KvaserCAN-Library/Sources/MacCAN
)

# Link against Qt, KvaserCAN and MacOS Framework
target_link_libraries(kvasercan PRIVATE
    Qt6::Core
    Qt6::SerialBus
    "-framework CoreFoundation"
    "-framework IOKit"
    ${CMAKE_CURRENT_SOURCE_DIR}/KvaserCAN-Library/Libraries/CANAPI/libUVCANKVL.a
    ${CMAKE_CURRENT_SOURCE_DIR}/KvaserCAN-Library/Libraries/KvaserCAN/libKvaserCAN.a
)

# Uhh, not sure if this is really needed, but I don't think it could hurt
target_compile_definitions(kvasercan PRIVATE QTKVASERCAN_LIBRARY)

# Install
# TODO
