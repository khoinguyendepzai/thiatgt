cmake_minimum_required(VERSION 3.15)
project(BrainCar LANGUAGES CXX)

# ============================================================================
# C++ Standard
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# OpenCV
# ============================================================================
# core: cv::Mat | imgproc + calib3d: nắn méo (CameraUndistorter) | videoio: cv::VideoCapture (camera)
# imgcodecs: cv::imencode / cv::imwrite | highgui: cv::imshow (dùng trong main)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d videoio imgcodecs highgui)

# ============================================================================
# Threading Support
# ============================================================================
find_package(Threads REQUIRED)

# ============================================================================
# Source Files
# ============================================================================
# Messages
set(MESSAGES_SOURCES
    ${PROJECT_SOURCE_DIR}/src/utils/messages/allMessages.cpp
    ${PROJECT_SOURCE_DIR}/src/utils/messages/messageHandlerSender.cpp
    ${PROJECT_SOURCE_DIR}/src/utils/messages/messageHandlerSubscriber.cpp
)

# Templates
set(TEMPLATES_SOURCES
    ${PROJECT_SOURCE_DIR}/src/templates/threadWithStop.cpp
)

# Gateway
set(GATEWAY_SOURCES
    ${PROJECT_SOURCE_DIR}/src/gateway/threadGateWay.cpp
)

# Hardware: serial
set(HARDWARE_SOURCES
    ${PROJECT_SOURCE_DIR}/src/hardware/serialhandler/serialPort.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/serialhandler/messageConverter.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/serialhandler/threadRead.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/serialhandler/threadWrite.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/serialhandler/threadSerialHandler.cpp
)

# Hardware: camera (ThreadCameraHandler: một thread đọc cả hai camera, nắn méo camera bám làn)
set(CAMERA_SOURCES
    ${PROJECT_SOURCE_DIR}/src/hardware/camera/frameBuffer.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/camera/threadCameraHandler.cpp
)

# Perception: nắn méo thấu kính (dò làn chạy trên PC, main.py)
set(PERCEPTION_SOURCES
    ${PROJECT_SOURCE_DIR}/src/perception/cameraUndistorter.cpp
)

# Network: UDP server, TCP client, handler quản lý cả hai
set(NETWORK_SOURCES
    ${PROJECT_SOURCE_DIR}/src/network/messageConverter.cpp
    ${PROJECT_SOURCE_DIR}/src/network/tcpClient.cpp
    ${PROJECT_SOURCE_DIR}/src/network/threadTcpClient.cpp
    ${PROJECT_SOURCE_DIR}/src/network/threadNetworkHandler.cpp
    ${PROJECT_SOURCE_DIR}/src/network/udpServer.cpp
    ${PROJECT_SOURCE_DIR}/src/network/threadUdpServer.cpp
)

# State Machine
set(STATEMACHINE_SOURCES
    ${PROJECT_SOURCE_DIR}/src/statemachine/systemMode.cpp
    ${PROJECT_SOURCE_DIR}/src/statemachine/stateMachine.cpp
)

# Control: ThreadControlHandler chỉ subscribe StateChange, phần điều khiển code sau (không còn bám làn / đổi làn)
set(CONTROL_SOURCES
    ${PROJECT_SOURCE_DIR}/src/control/threadControlHandler.cpp
)

# Combine all sources except main
set(LIB_SOURCES
    ${MESSAGES_SOURCES}
    ${TEMPLATES_SOURCES}
    ${GATEWAY_SOURCES}
    ${HARDWARE_SOURCES}
    ${CAMERA_SOURCES}
    ${PERCEPTION_SOURCES}
    ${NETWORK_SOURCES}
    ${STATEMACHINE_SOURCES}
    ${CONTROL_SOURCES}
)

# ============================================================================
# Library + Executable
# ============================================================================
# Include dir, OpenCV và Threads là PUBLIC nên braincar (main) tự kế thừa qua braincar_lib
add_library(braincar_lib STATIC ${LIB_SOURCES})
target_include_directories(braincar_lib PUBLIC ${PROJECT_SOURCE_DIR}/inc ${OpenCV_INCLUDE_DIRS})
target_link_libraries(braincar_lib PUBLIC ${OpenCV_LIBS} Threads::Threads)

add_executable(braincar main.cpp)
target_link_libraries(braincar PRIVATE braincar_lib)

# ============================================================================
# Compiler Flags
# ============================================================================
# -Wall -Wextra: bắt lỗi thứ tự khởi tạo member (-Wreorder), biến không dùng... (chỉ cảnh báo)
foreach(target braincar braincar_lib)
    target_compile_options(${target} PRIVATE -Wall -Wextra)
    if(CMAKE_BUILD_TYPE MATCHES Debug)
        target_compile_options(${target} PRIVATE -g -O0)
    else()
        target_compile_options(${target} PRIVATE -O2)
    endif()
endforeach()

# ============================================================================
# Summary
# ============================================================================
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "OpenCV: ${OpenCV_VERSION}")
message(STATUS "Source Directory: ${PROJECT_SOURCE_DIR}")
message(STATUS "Binary Directory: ${CMAKE_BINARY_DIR}")
