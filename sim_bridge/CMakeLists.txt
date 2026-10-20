add_library(sim_bridge STATIC simulator.cpp foxglove_debug.cpp)
target_include_directories(sim_bridge PUBLIC ${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(sim_bridge PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json
  Threads::Threads Boost::system ${OpenCV_LIBS})
