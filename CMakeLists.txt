cmake_minimum_required(VERSION 3.0.2)
project(jupiter_juno)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  rqt_gui
  rqt_gui_py
)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    cv_bridge
    rqt_gui
    rqt_gui_py
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python scripts
catkin_install_python(PROGRAMS
  jupiter_juno/eye_detector_node.py
  jupiter_juno/alert_system_node.py
  jupiter_juno/tts_node.py
  jupiter_juno/speech_recognition_node.py
  jupiter_juno/gemini_conversation_node.py
  jupiter_juno/jupiter_juno_monitor.py
  standalone_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install model files
install(FILES
  jupiter_juno/shape_predictor_68_face_landmarks.dat
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

# Install plugin files
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 