add_subdirectory(planner/tinympc)
add_library(auto_aim STATIC armor.cpp solver.cpp target.cpp tracker.cpp
  planner/planner.cpp shooter.cpp yolo.cpp yolos/yolov5.cpp)
target_compile_definitions(auto_aim PUBLIC OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)
target_link_libraries(auto_aim PUBLIC tools yaml-cpp tinympcstatic openvino::runtime)
