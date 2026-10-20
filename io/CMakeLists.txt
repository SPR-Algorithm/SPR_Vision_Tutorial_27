cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

add_subdirectory(serial)

add_library(io STATIC
  hikrobot/hikrobot.cpp
  camera.cpp
  gimbal/gimbal.cpp
)

target_include_directories(io PUBLIC
  ${PROJECT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/include
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(HIKROBOT_SDK_ARCH amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(HIKROBOT_SDK_ARCH arm64)
else()
  message(FATAL_ERROR "Unsupported HikRobot SDK architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(HIKROBOT_SDK_LIBRARY
  "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/${HIKROBOT_SDK_ARCH}/libMvCameraControl.so")
if(NOT EXISTS "${HIKROBOT_SDK_LIBRARY}")
  message(FATAL_ERROR "HikRobot SDK library not found: ${HIKROBOT_SDK_LIBRARY}")
endif()

target_link_libraries(io PUBLIC
  tools
  Eigen3::Eigen
  ${OpenCV_LIBS}
  yaml-cpp
  serial
  PkgConfig::LIBUSB
  "${HIKROBOT_SDK_LIBRARY}"
)
