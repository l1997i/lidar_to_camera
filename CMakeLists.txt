cmake_minimum_required(VERSION 3.0)

project(camera_fusion)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(PCL 1.11 REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executables for exercises
add_executable (show_lidar_top_view src/show_lidar_top_view.cpp src/structIO.cpp)
target_link_libraries (show_lidar_top_view ${OpenCV_LIBRARIES})

add_executable (project_lidar_to_camera src/project_lidar_to_camera.cpp src/structIO.cpp)
target_link_libraries (project_lidar_to_camera ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
