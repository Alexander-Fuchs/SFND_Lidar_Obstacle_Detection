cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

set(Qt5Core_DIR /opt/homebrew/Cellar/qt@5/5.15.8_2/lib/cmake/Qt5Core)
set(Qt5Gui_DIR	/opt/homebrew/Cellar/qt@5/5.15.8_2/lib/cmake/Qt5Gui)
set(Qt5OpenGL_DIR	/opt/homebrew/Cellar/qt@5/5.15.8_2/lib/cmake/Qt5OpenGL)
set(Qt5Widgets_DIR	/opt/homebrew/Cellar/qt@5/5.15.8_2/lib/cmake/Qt5Widgets)
set(Qt5_DIR	/opt/homebrew/lib/cmake/Qt5)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (quizRansac ransac2d.cpp ../../render/render.cpp)
target_link_libraries (quizRansac ${PCL_LIBRARIES})




