cmake_minimum_required(VERSION 3.0)
project(dfall_pkg)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  genmsg
  rosbag
  roslib
  )

find_package (Eigen3 3.3 NO_MODULE)


# VICON DATASTREAM SDK PACKAGE
# Ensure that the cache variable where the result would be stored is clear
message(STATUS "NOTE: Prior to being unset: VICON_LIBRARY = " ${VICON_LIBRARY})
unset(VICON_LIBRARY CACHE)
message(STATUS "NOTE: After being unset: VICON_LIBRARY = " ${VICON_LIBRARY})

# Find the Vicon Data Stream SDK Pacakge
find_library(VICON_LIBRARY ViconDataStreamSDK_CPP PATHS lib/vicon NO_DEFAULT_PATH)
message(STATUS "NOTE: After calling find_library(): VICON_LIBRARY = " ${VICON_LIBRARY})

# Let the user know if the library was found or not
if(VICON_LIBRARY)
	message(STATUS "NOTE: the Vicon Data Stream SDK library was found")
else()
	message(STATUS "NOTE: the Vicon Data Stream SDK library was NOT found")
endif()


# GUI -- Add precompiler definitions to include ROS things in GUI compilation
add_definitions(-DCATKIN_MAKE)

# GUI -- Things needed for Qt wrapper build process
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# GUI -- Add Qt support
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Svg REQUIRED)



# GUI -- Add src, includes, and resources
set(SYSTEM_CONFIG_GUI_LIB_PATH_SRC   ${PROJECT_SOURCE_DIR}/GUI_Qt/systemConfigGUI/src)
set(SYSTEM_CONFIG_GUI_LIB_PATH_INC   ${PROJECT_SOURCE_DIR}/GUI_Qt/systemConfigGUI/include)
set(SYSTEM_CONFIG_GUI_LIB_PATH_FORMS ${PROJECT_SOURCE_DIR}/GUI_Qt/systemConfigGUI/forms)
set(SYSTEM_CONFIG_GUI_RESOURCE_FILE_QRC  ${PROJECT_SOURCE_DIR}/GUI_Qt/systemConfigGUI/systemconfiggui.qrc)



# Flying Agent GUI -- Add src, includes, forms, and resources
set(FLYING_AGENT_GUI_LIB_PATH_SRC      ${PROJECT_SOURCE_DIR}/GUI_Qt/flyingAgentGUI/src)
set(FLYING_AGENT_GUI_LIB_PATH_INC      ${PROJECT_SOURCE_DIR}/GUI_Qt/flyingAgentGUI/include)
set(FLYING_AGENT_GUI_LIB_PATH_FORMS    ${PROJECT_SOURCE_DIR}/GUI_Qt/flyingAgentGUI/forms)
set(FLYING_AGENT_GUI_RESOURCE_FILE_QRC ${PROJECT_SOURCE_DIR}/GUI_Qt/flyingAgentGUI/flyingagentgui.qrc)



# #set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# #set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# GUI -- need c++11 for some things
add_definitions(-std=c++11)



# GUI -- Special Qt sources that need to be wrapped before being compiled
# they have the Qt macro QOBJECT inside, the MOC cpp file needs to be done manually
set(SRC_HDRS_QOBJECT_GUI
  ${SYSTEM_CONFIG_GUI_LIB_PATH_INC}/crazyFlyZoneTab.h
  ${SYSTEM_CONFIG_GUI_LIB_PATH_INC}/myGraphicsScene.h
  ${SYSTEM_CONFIG_GUI_LIB_PATH_INC}/myGraphicsView.h
  ${SYSTEM_CONFIG_GUI_LIB_PATH_INC}/mainguiwindow.h
  ${SYSTEM_CONFIG_GUI_LIB_PATH_INC}/rosNodeThread_for_systemConfigGUI.h
  ${SYSTEM_CONFIG_GUI_LIB_PATH_INC}/CFLinker.h
  )
# GUI -- wrap UI file and QOBJECT files
qt5_wrap_ui(UIS_HDRS_GUI ${SYSTEM_CONFIG_GUI_LIB_PATH_FORMS}/mainguiwindow.ui)
qt5_wrap_cpp(SRC_MOC_HDRS_GUI ${SRC_HDRS_QOBJECT_GUI})
# GUI -- wrap resource file qrc->rcc
qt5_add_resources(SYSTEM_CONFIG_GUI_RESOURCE_FILE_RRC ${SYSTEM_CONFIG_GUI_RESOURCE_FILE_QRC})




# Flying Agent GUI
# - Special Qt sources that need to be wrapped before being compiled
#   they have the Qt macro QOBJECT inside, the MOC cpp file needs to
#   be done manually
set(SRC_HDRS_QOBJECT_FLYING_AGENT_GUI
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/connectstartstopbar.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/controllertabs.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/coordinator.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/coordinatorrow.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/defaultcontrollertab.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/enablecontrollerloadyamlbar.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/mainwindow.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/pickercontrollertab.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/rosNodeThread_for_flyingAgentGUI.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/safecontrollertab.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/studentcontrollertab.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/templatecontrollertab.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/topbanner.h
  ${FLYING_AGENT_GUI_LIB_PATH_INC}/tuningcontrollertab.h
  )
# Flying Agent GUI -- wrap UI file and QOBJECT files
qt5_wrap_ui(UIS_HDRS_FLYING_AGENT_GUI
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/connectstartstopbar.ui
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/controllertabs.ui
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/coordinator.ui
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/coordinatorrow.ui
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/defaultcontrollertab.ui
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/enablecontrollerloadyamlbar.ui
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/mainwindow.ui
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/pickercontrollertab.ui
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/safecontrollertab.ui
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/studentcontrollertab.ui
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/templatecontrollertab.ui
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/topbanner.ui
  ${FLYING_AGENT_GUI_LIB_PATH_FORMS}/tuningcontrollertab.ui
  )
qt5_wrap_cpp(SRC_MOC_HDRS_FLYING_AGENT_GUI ${SRC_HDRS_QOBJECT_FLYING_AGENT_GUI})
# Flying Agent GUI -- wrap resource file qrc->rcc
qt5_add_resources(FLYING_AGENT_GUI_RESOURCE_FILE_RCC ${FLYING_AGENT_GUI_RESOURCE_FILE_QRC})



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_message_files(
  FILES
  UnlabeledMarker.msg
  CrazyflieData.msg
  ViconData.msg
  ControlCommand.msg
  CrazyflieContext.msg
  AreaBounds.msg
  Setpoint.msg
  SetpointV2.msg
  CrazyflieEntry.msg
  CrazyflieDB.msg
  #------------------------
  IntWithHeader.msg
  FloatWithHeader.msg
  StringWithHeader.msg
  SetpointWithHeader.msg
  CustomButtonWithHeader.msg
  #------------------------
  DebugMsg.msg
  CustomButton.msg
  ViconSubscribeObjectName.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
add_service_files(
  FILES
  IntIntService.srv
  Controller.srv
  CMRead.srv
  CMQuery.srv
  CMUpdate.srv
  CMCommand.srv
  LoadYamlFromFilename.srv
  GetSetpointService.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${SYSTEM_CONFIG_GUI_LIB_PATH_INC}            # GUI -- include headers from GUI in package
  INCLUDE_DIRS include ${FLYING_AGENT_GUI_LIB_PATH_INC}  # FlyingAgentGUI -- include headers from GUI in package
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs rosbag roslib
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${SYSTEM_CONFIG_GUI_LIB_PATH_INC}    # SystemConfigGUI -- include directory inside GUI folder
  ${FLYING_AGENT_GUI_LIB_PATH_INC}     # FlyingAgentGUI  -- include directory inside GUI folder
  ${catkin_INCLUDE_DIRS}
  include
  include/nodes
)

## Declare a C++ library
##add_library(${PROJECT_NAME}
##  src/blablalba.cpp
##)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/dfall_pkg_node.cpp)

if(VICON_LIBRARY)
	add_executable(ViconDataPublisher       src/nodes/ViconDataPublisher.cpp)
endif()

add_executable(FlyingAgentClient         src/nodes/FlyingAgentClient.cpp
                                         src/classes/GetParamtersAndNamespaces.cpp)
add_executable(BatteryMonitor            src/nodes/BatteryMonitor.cpp
                                         src/classes/GetParamtersAndNamespaces.cpp)
add_executable(DefaultControllerService  src/nodes/DefaultControllerService.cpp
                                         src/classes/GetParamtersAndNamespaces.cpp)
add_executable(SafeControllerService     src/nodes/SafeControllerService.cpp)
add_executable(DemoControllerService     src/nodes/DemoControllerService.cpp
                                         src/classes/GetParamtersAndNamespaces.cpp)
add_executable(StudentControllerService  src/nodes/StudentControllerService.cpp
                                         src/classes/GetParamtersAndNamespaces.cpp)
add_executable(MpcControllerService      src/nodes/MpcControllerService.cpp)
add_executable(RemoteControllerService   src/nodes/RemoteControllerService.cpp)
add_executable(TuningControllerService   src/nodes/TuningControllerService.cpp
                                         src/classes/GetParamtersAndNamespaces.cpp)
add_executable(PickerControllerService   src/nodes/PickerControllerService.cpp
                                         src/classes/GetParamtersAndNamespaces.cpp)
add_executable(TemplateControllerService src/nodes/TemplateControllerService.cpp
                                         src/classes/GetParamtersAndNamespaces.cpp)
add_executable(CentralManagerService     src/nodes/CentralManagerService.cpp src/CrazyflieIO.cpp)
add_executable(ParameterService          src/nodes/ParameterService.cpp)



# GUI -- Add sources here
set(SYSTEM_CONFIG_GUI_CPP_SOURCES              # compilation of sources
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/mainguiwindow.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/main.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/cornergrabber.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/crazyFlyZone.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/crazyFlyZoneTab.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/myGraphicsRectItem.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/myGraphicsScene.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/myGraphicsView.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/tablePiece.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/marker.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/rosNodeThread_for_systemConfigGUI.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/crazyFly.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/CFLinker.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/channelLUT.cpp
    ${SYSTEM_CONFIG_GUI_LIB_PATH_SRC}/centerMarker.cpp
    )




# FLYING AGENT GUI -- Add sources here
set(FLYING_AGENT_GUI_CPP_SOURCES         # compilation of sources
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/mainwindow.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/main.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/rosNodeThread_for_flyingAgentGUI.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/connectstartstopbar.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/controllertabs.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/coordinator.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/coordinatorrow.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/defaultcontrollertab.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/enablecontrollerloadyamlbar.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/pickercontrollertab.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/safecontrollertab.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/studentcontrollertab.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/templatecontrollertab.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/topbanner.cpp
    ${FLYING_AGENT_GUI_LIB_PATH_SRC}/tuningcontrollertab.cpp
    )



# GUI -- Add executables here
add_executable(systemConfigGUI ${SYSTEM_CONFIG_GUI_CPP_SOURCES} ${UIS_HDRS_GUI} ${SRC_MOC_HDRS_GUI} ${SYSTEM_CONFIG_GUI_RESOURCE_FILE_RRC})
qt5_use_modules(systemConfigGUI Widgets)



# FLYING AGENT GUI -- Add executables here
add_executable(flyingAgentGUI ${FLYING_AGENT_GUI_CPP_SOURCES} ${UIS_HDRS_FLYING_AGENT_GUI} ${SRC_MOC_HDRS_FLYING_AGENT_GUI} ${FLYING_AGENT_GUI_RESOURCE_FILE_RCC})
qt5_use_modules(flyingAgentGUI Widgets)



if(VICON_LIBRARY)
	add_dependencies(ViconDataPublisher       dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
endif()

add_dependencies(FlyingAgentClient         dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(BatteryMonitor            dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(DefaultControllerService  dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(SafeControllerService     dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(DemoControllerService     dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(StudentControllerService  dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(MpcControllerService      dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(RemoteControllerService   dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(TuningControllerService   dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(PickerControllerService   dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(TemplateControllerService dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(CentralManagerService     dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(ParameterService          dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})



# GUI-- dependencies
add_dependencies(systemConfigGUI dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})



# FLYING AGENT GUI-- dependencies
add_dependencies(flyingAgentGUI dfall_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})



## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# find_library(VICON_LIBRARY ViconDataStreamSDK_CPP HINTS lib/vicon)

if(VICON_LIBRARY)
	target_link_libraries(ViconDataPublisher ${catkin_LIBRARIES})
	target_link_libraries(ViconDataPublisher ${VICON_LIBRARY})
endif()

target_link_libraries(FlyingAgentClient         ${catkin_LIBRARIES})
target_link_libraries(BatteryMonitor            ${catkin_LIBRARIES})
target_link_libraries(DefaultControllerService  ${catkin_LIBRARIES})
target_link_libraries(SafeControllerService     ${catkin_LIBRARIES})
target_link_libraries(DemoControllerService     ${catkin_LIBRARIES})
target_link_libraries(StudentControllerService  ${catkin_LIBRARIES})
if(Eigen3_FOUND)
  target_link_libraries(MpcControllerService     ${catkin_LIBRARIES} Eigen3::Eigen)
else()
  target_link_libraries(MpcControllerService     ${catkin_LIBRARIES})
endif()
target_link_libraries(RemoteControllerService   ${catkin_LIBRARIES})
target_link_libraries(TuningControllerService   ${catkin_LIBRARIES})
target_link_libraries(PickerControllerService   ${catkin_LIBRARIES})
target_link_libraries(TemplateControllerService ${catkin_LIBRARIES})
target_link_libraries(CentralManagerService     ${catkin_LIBRARIES})
target_link_libraries(ParameterService          ${catkin_LIBRARIES})



# GUI -- link libraries
target_link_libraries(systemConfigGUI Qt5::Widgets) # GUI -- let systemConfigGUI have acesss to Qt stuff
target_link_libraries(systemConfigGUI Qt5::Svg)
target_link_libraries(systemConfigGUI ${catkin_LIBRARIES})



# Flying Agent GUI -- link libraries
target_link_libraries(flyingAgentGUI Qt5::Widgets) # GUI -- let flyingAgentGUI have acesss to Qt stuff
target_link_libraries(flyingAgentGUI ${catkin_LIBRARIES})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dfall_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
