cmake_minimum_required(VERSION 3.16)

# 需要手动指定 OpenVINO 时，取消下面这行的注释
# set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(auto_aim OBJECT 
    armor.cpp
    yolo.cpp
    yolos/yolov5.cpp
)

target_link_libraries(auto_aim io openvino::runtime )