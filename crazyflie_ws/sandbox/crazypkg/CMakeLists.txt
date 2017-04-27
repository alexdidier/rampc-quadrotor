cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()



find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

#set(CMAKE_PREFIX_PATH "/home/crazyfly/Qt5.7.0/5.7/gcc_64")
#set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )
#add_definitions(${Qt5Widgets_DEFINITIONS})


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

qt5_wrap_ui(UIS_HDRS gui/untitled/mainguiwindow.ui)
qt5_wrap_cpp(SRC_HDRS gui/untitled/mainguiwindow.h)


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


link_directories(${PROJECT_SOURCE_DIR}/src/ViconLibs)
#link_directories(${PROJECT_SOURCE_DIR}/src)
#INCLUDE_DIRECTORIES(/usr/include/qt4/QtGui)
#INCLUDE_DIRECTORIES(/usr/include/qt4)
#include_directories(${PROJECT_SOURCE_DIR}/include)
#include_directories(${PROJECT_SOURCE_DIR}/gui/untitled)
#include_directories(${Qt5Widgets_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS})

#link_directories(/home/crazyfly/Qt5.7.0/5.7/gcc_64/include/QtWidgets)
#link_directories(/home/crazyfly/Qt5.7.0/5.7/gcc_64/include)



rosbuild_add_executable(ViconDataStreamSDK nodes/ViconDataStreamSDK.cpp)
rosbuild_add_executable(FlightControl nodes/FlightControl.cpp)
rosbuild_add_executable(GUI gui/untitled/mainguiwindow.cpp gui/untitled/main.cpp ${UIS_HDRS} ${SRC_HDRS})
qt5_use_modules(GUI Widgets)

rosbuild_add_library(CrazyControl src/ControlManager.cpp)
rosbuild_add_library(CrazyControllers src/CrazyFlieControllers.cpp)
#rosbuild_add_library(GUIMain gui/untitled/mainguiwindow.cpp)

target_link_libraries(FlightControl CrazyControl)
target_link_libraries(FlightControl CrazyControllers)
#target_link_libraries(GUI GUIMain)
target_link_libraries(GUI Qt5::Widgets)
#target_link_libraries(GUI CrazyControl)
#target_link_libraries(GUI CrazyControllers)
target_link_libraries(ViconDataStreamSDK libViconDataStreamSDK_CPP.so)
