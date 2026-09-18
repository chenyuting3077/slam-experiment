cmake_minimum_required(VERSION 3.8)
project(slam_comparison)
find_package(ament_cmake REQUIRED)
find_package(cartographer REQUIRED)

add_executable(pbstream_to_tum src/pbstream_to_tum.cc)
target_link_libraries(pbstream_to_tum cartographer)
install(TARGETS pbstream_to_tum DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch scripts DESTINATION share/${PROJECT_NAME})
ament_package()
