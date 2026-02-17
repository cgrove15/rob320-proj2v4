cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 20)

project(click-to-drive)

# Used to find GTest on CAEN
list(APPEND CMAKE_PREFIX_PATH "~/.local")

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

# Used to compile on Autograder (possibly unnecessary ...)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (NOT DEFINED ARCH)
    # If we do not pre-define our architecture, run this command to infer it
    EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCH )
    # Normalize name
    if (ARCH STREQUAL "arm64")
        set(ARCH "aarch64")
    endif()
endif()

if (NOT DEFINED OS)
    # If we do not pre-define our operating system, infer it
    if(APPLE)
        set(OS "macos")
    elseif(UNIX)
        set(OS "linux")
    else()
        message( FATAL_ERROR "Unsupported OS." )
    endif()
endif()

message( STATUS "Architecture: ${ARCH}" )
message( STATUS "Operating System: ${OS}" )

# Compile the Project 2 library
add_library(project2 src/rix/ipc/client_tcp.cpp
    src/rix/ipc/server_tcp.cpp
    src/rix/ipc/connection_tcp.cpp
    src/rix/ipc/endpoint.cpp
    src/rix/ipc/socket.cpp
)
target_include_directories(project2 PRIVATE include/)

# Link against the proper Project 1 library
target_link_libraries(project2 PUBLIC ${CMAKE_SOURCE_DIR}/lib/${OS}-${ARCH}/libproject1.a)

# Build the RPLIDAR SDK
if(APPLE)
    set(OS_DIR_NAME "macOS")
    add_compile_definitions(_MACOS)
elseif(UNIX)
    set(OS_DIR_NAME "linux")
elseif(WIN32)
    set(OS_DIR_NAME "win32")
    add_compile_definitions(_WIN32)
endif()

set(RPLIDAR_SDK_PATH "./rplidar_sdk/sdk/")
file(GLOB RPLIDAR_SDK_SRC "${RPLIDAR_SDK_PATH}/src/arch/${OS_DIR_NAME}/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp")

add_library(rplidar_sdk STATIC ${RPLIDAR_SDK_SRC})
target_include_directories(rplidar_sdk PUBLIC ${RPLIDAR_SDK_PATH}/include ${RPLIDAR_SDK_PATH}/src)
target_link_libraries(rplidar_sdk Threads::Threads)

# Compile the Socket test
add_executable(socket_test tests/socket.cpp)
target_link_libraries(socket_test PRIVATE project2 GTest::gtest_main)
target_include_directories(socket_test PRIVATE include/)

# Compile the ClientTCP, ConnectionTCP, and ServerTCP test
add_executable(tcp_test tests/tcp.cpp)
target_link_libraries(tcp_test PRIVATE project2 GTest::gtest_main GTest::gmock)
target_include_directories(tcp_test PRIVATE include/)

# Compile the MBot library
add_library(mbot src/mbot/mbot.cpp)
target_link_libraries(mbot Threads::Threads)
target_include_directories(mbot PRIVATE include/)

# Compile the Lidar library
add_library(lidar src/lidar/lidar.cpp)
target_link_libraries(lidar PUBLIC rplidar_sdk)
target_include_directories(lidar PRIVATE include/)

# Compile the Lidar Driver executable
add_executable(lidar_driver src/lidar_driver/lidar_driver.cpp src/lidar_driver/main.cpp)
target_link_libraries(lidar_driver PRIVATE project2 lidar)
target_include_directories(lidar_driver PRIVATE include/)

# Compile the MBot Driver executable
add_executable(mbot_driver src/mbot_driver/mbot_driver.cpp src/mbot_driver/main.cpp)
target_link_libraries(mbot_driver PRIVATE project2 mbot)
target_include_directories(mbot_driver PRIVATE include/)

# Compile the MBot Driver test
add_executable(mbot_driver_test tests/mbot_driver.cpp src/mbot_driver/mbot_driver.cpp)
target_link_libraries(mbot_driver_test PRIVATE project2 GTest::gtest_main GTest::gmock)
target_include_directories(mbot_driver_test PRIVATE include/)

# Compile the Lidar Driver test
add_executable(lidar_driver_test tests/lidar_driver.cpp src/lidar_driver/lidar_driver.cpp)
target_link_libraries(lidar_driver_test PRIVATE project2 lidar GTest::gtest_main GTest::gmock)
target_include_directories(lidar_driver_test PRIVATE include/)
