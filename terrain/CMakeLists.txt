cmake_minimum_required(VERSION 3.8.2)

project(Terrain)
find_package(OpenCV REQUIRED)
find_package(OpenGL)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL Gui OpenGLExtensions)
find_package(ZLIB REQUIRED)

include_directories(${ZLIB_INCLUDE_DIRS})
include_directories(/usr/local/include/eigen3)
include_directories(${CMAKE_CURRENT_LIST_DIR}/src)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -w")

find_package(OpenMP REQUIRED)

add_executable(
	Canvas
	src/Drawer.h
	src/Drawer.cpp
	src/FlowField.h
	src/FlowField.cpp
	src/ImageWidget.h
	src/ImageWidget.cpp
	src/MainWindow.h
	src/MainWindow.cpp
	src/MainWindow.ui
	src/TensorField.h
	src/TensorField.cpp
	src/RoadGenerator.h
	src/RoadGenerator.cpp
	src/test_canvas.cpp
)

target_link_libraries(
	Canvas
	Qt5::Widgets
	${OpenCV_LIBS}
	OpenMP::OpenMP_CXX
)