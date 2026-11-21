add_library(tools STATIC
  exiter.cpp extended_kalman_filter.cpp img_tools.cpp math_tools.cpp
  trajectory.cpp logger.cpp crc.cpp pid.cpp plotter.cpp recorder.cpp ransac_sine_fitter.cpp)
target_include_directories(tools PUBLIC ${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tools PUBLIC Eigen3::Eigen fmt::fmt spdlog::spdlog
  nlohmann_json::nlohmann_json Threads::Threads ${OpenCV_LIBS})
