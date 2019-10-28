cmake_minimum_required(VERSION 2.8)

project(MultiCameraTriangulationCeres)
find_package( OpenCV REQUIRED ) 
find_package(Ceres REQUIRED PATHS $ENV{Ceres_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include_directories("../MultiCameraReconstructFromRecog/nlohmann")

add_compile_definitions(_ENABLE_EXTENDED_ALIGNED_STORAGE)
include_directories($ENV{AC_DIR})
# Project
add_executable(MultiCameraTriangulationCeres main.cpp)
target_link_libraries(MultiCameraTriangulationCeres  ${CERES_LIBRARIES})
target_link_libraries( MultiCameraTriangulationCeres ${OpenCV_LIBS} )  