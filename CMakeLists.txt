cmake_minimum_required(VERSION 3.20)
project(yoctoLidar VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra)

if(NOT EXISTS ${CMAKE_SOURCE_DIR}/rplidar_sdk)
    include(FetchContent)
    FetchContent_Declare(
        rplidar_sdk
        GIT_REPOSITORY https://github.com/Slamtec/rplidar_sdk.git
        GIT_TAG        master
        SOURCE_DIR     ${CMAKE_SOURCE_DIR}/rplidar_sdk
    )
    FetchContent_MakeAvailable(rplidar_sdk)
endif()

set(RPLIDAR_SDK_PATH ${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk)

set(RPLIDAR_SOURCES
    ${RPLIDAR_SDK_PATH}/src/rplidar_driver.cpp
    ${RPLIDAR_SDK_PATH}/src/sl_lidar_driver.cpp
    ${RPLIDAR_SDK_PATH}/src/sl_serial_channel.cpp
    ${RPLIDAR_SDK_PATH}/src/sl_tcp_channel.cpp
    ${RPLIDAR_SDK_PATH}/src/sl_udp_channel.cpp
    ${RPLIDAR_SDK_PATH}/src/sl_async_transceiver.cpp
    ${RPLIDAR_SDK_PATH}/src/sl_crc.cpp
    ${RPLIDAR_SDK_PATH}/src/sl_lidarprotocol_codec.cpp
    ${RPLIDAR_SDK_PATH}/src/dataunpacker/dataunpacker.cpp
    ${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/handler_capsules.cpp
    ${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/handler_hqnode.cpp
    ${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/handler_normalnode.cpp
    ${RPLIDAR_SDK_PATH}/src/hal/thread.cpp
    ${RPLIDAR_SDK_PATH}/src/arch/linux/net_serial.cpp
    ${RPLIDAR_SDK_PATH}/src/arch/linux/net_socket.cpp
    ${RPLIDAR_SDK_PATH}/src/arch/linux/timer.cpp
)


add_executable(yoctoLidar
    src/main.cpp
    src/obstacleDetection.cpp
    ${RPLIDAR_SOURCES}
)

target_include_directories(yoctoLidar PRIVATE
    include
    ${RPLIDAR_SDK_PATH}/include
    ${RPLIDAR_SDK_PATH}/include/hal
    ${RPLIDAR_SDK_PATH}/src
    ${RPLIDAR_SDK_PATH}/src/arch/linux
)

target_link_libraries(yoctoLidar PRIVATE pthread)