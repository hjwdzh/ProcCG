cmake_minimum_required(VERSION 3.8.2)

project(Terrain)
find_package(OpenCV REQUIRED)
find_package(OpenGL)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL Gui OpenGLExtensions)
find_package(ZLIB REQUIRED)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_BUILD_TYPE "debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g")
elseif(CMAKE_BUILD_TYPE MATCHES "Release")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++14 -mavx -fPIC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -mavx -fPIC")
endif()

include_directories(${ZLIB_INCLUDE_DIRS})
include_directories(/usr/local/include/eigen3)
include_directories(${CMAKE_CURRENT_LIST_DIR}/src)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -w")

find_package(OpenMP REQUIRED)
find_package(CGAL)
include(${CGAL_USE_FILE})

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
	src/RoadGenerator.h
	src/RoadGenerator.cpp
	src/TensorField.h
	src/TensorField.cpp
	src/Terrain.h
	src/Terrain.cpp
	src/Triangulate.h
	src/Triangulate.cpp
	src/main.cpp
)

target_link_libraries(
	Canvas
	Qt5::Widgets
	${OpenCV_LIBS}
	OpenMP::OpenMP_CXX
)
