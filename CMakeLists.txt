cmake_minimum_required(VERSION 3.16)

project(ModBeeGsm
    VERSION 0.1.0
    DESCRIPTION "AT-command cellular modems for Arduino and hosts - that never block"
    LANGUAGES CXX)

# C++17 for THIS build - the host examples, and the test suite where present,
# use it.
#
# THE LIBRARY ITSELF NEEDS NOTHING NEWER THAN C++11, and that is deliberate:
# several Arduino cores this library claims to support compile at
# -std=gnu++11, including SAMD. A library that needed C++17 would not build on
# the boards in library.properties. Keep it that way: one `if constexpr`
# written on a host breaks every one of them, silently, until somebody builds
# for a board.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(GSM_BUILD_EXAMPLES  "Build the plain C++ examples" ON)
option(GSM_BUILD_TESTS     "Build the test suite, when present" ON)
option(GSM_SANITIZE        "Build with the address and undefined behaviour sanitizers" OFF)
option(GSM_SANITIZE_THREAD "Build with the thread sanitizer (excludes GSM_SANITIZE)" OFF)
option(GSM_NO_EXCEPTIONS   "Build without exceptions or RTTI, as a wrapper would" OFF)

# NO FEATURE OPTIONS, and no way to select a modem at build time.
#
# Selecting the modem with the preprocessor is the single decision that makes
# TinyGSM unusable here: one firmware drives one family, and a host build that
# adapts to whatever is on /dev/ttyUSB0 cannot be written at all. Every driver
# compiles, always, and which one is used is decided at run time from ATI. A
# board that only ever meets one module pays for the others in unreferenced
# classes, which --gc-sections drops.

# The library parses bytes it did not produce, over a link that can corrupt
# them, at a baud rate that might be wrong. An out-of-bounds read on a
# malformed reply is therefore the failure mode that matters most and the one
# least likely to surface as a wrong answer. Run the suite under these before
# a release.
if(GSM_SANITIZE)
    if(MSVC)
        add_compile_options(/fsanitize=address)
    else()
        add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -g)
        add_link_options(-fsanitize=address,undefined)
    endif()
endif()

# A host runtime may drive one modem from two threads: one task servicing the
# link while application code opens a socket from a slower one. The address
# sanitizer says nothing about that race; only the thread sanitizer does, and
# the two cannot be built together.
if(GSM_SANITIZE_THREAD)
    if(GSM_SANITIZE)
        message(FATAL_ERROR "GSM_SANITIZE and GSM_SANITIZE_THREAD cannot be combined")
    endif()
    if(NOT MSVC)
        add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g)
        add_link_options(-fsanitize=thread)
    endif()
endif()

# Generated IEC 61131 function-block code, and several MCU toolchains, build
# with both of these off. Proving the library compiles that way is cheaper than
# discovering it does not once a wrapper exists.
if(GSM_NO_EXCEPTIONS AND NOT MSVC)
    add_compile_options(-fno-exceptions -fno-rtti)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# =============================================================================
# LIBRARY
# =============================================================================

# Listed explicitly rather than globbed, so adding a file is a deliberate act
# and CMake never silently misses one.
set(GSM_SOURCES
    src/GsmPlatform.cpp
    src/GsmResult.cpp
    src/GsmAtParser.cpp
    src/GsmParse.cpp
    src/GsmSmsPdu.cpp
    src/GsmSms.cpp
    src/GsmGnss.cpp
    src/GsmClock.cpp
    src/GsmUrcRouter.cpp
    src/GsmAtEngine.cpp
    src/GsmTypes.cpp
    src/GsmDriverRegistry.cpp
    src/GsmModem.cpp
    src/GsmSocket.cpp
    src/GsmClient.cpp
    src/GsmUDP.cpp
    src/GsmDevice.cpp
    src/drivers/GsmBootstrap.cpp
    src/drivers/GsmDriver3gpp.cpp
    src/drivers/GsmDriverA76XX.cpp
    src/drivers/GsmDriverSIM707x.cpp
    src/transport/GsmSerialTransport.cpp
)

add_library(modbee-gsm STATIC ${GSM_SOURCES})

# The minimum a consumer needs, stated on the target rather than only in a
# comment. Anything linking this may be built at a newer standard; it may not
# be built at an older one.
target_compile_features(modbee-gsm PUBLIC cxx_std_11)
add_library(ModBee::gsm ALIAS modbee-gsm)

target_include_directories(modbee-gsm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/modbee>
)

if(WIN32)
    target_link_libraries(modbee-gsm PUBLIC ws2_32)
else()
    target_link_libraries(modbee-gsm PUBLIC Threads::Threads)
endif()

if(MSVC)
    target_compile_options(modbee-gsm PRIVATE /W4)
else()
    target_compile_options(modbee-gsm PRIVATE -Wall -Wextra)
endif()

# =============================================================================
# EXAMPLES
# =============================================================================

if(GSM_BUILD_EXAMPLES AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/cpp/CMakeLists.txt)
    add_subdirectory(examples/cpp)
endif()

# =============================================================================
# OPTIONAL SUBDIRECTORIES
# =============================================================================
#
# Guarded by EXISTS, not just by the option: these directories are part of how
# the library is developed and are not distributed with it, so a clean checkout
# configures and builds with none of them present.

option(GSM_BUILD_TOOLS "Build the host-side tools, when present" ON)

if(GSM_BUILD_TOOLS AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tools/CMakeLists.txt)
    add_subdirectory(tools)
endif()

if(GSM_BUILD_TESTS AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt)
    enable_testing()
    add_subdirectory(test)
endif()
