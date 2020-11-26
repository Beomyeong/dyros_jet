# Install script for directory: /home/beom/catkin_ws/src/mujoco_ros_sim/mujoco_ros_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/beom/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mujoco_ros_msgs/msg" TYPE FILE FILES
    "/home/beom/catkin_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointInit.msg"
    "/home/beom/catkin_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointSet.msg"
    "/home/beom/catkin_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/JointState.msg"
    "/home/beom/catkin_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorState.msg"
    "/home/beom/catkin_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SensorBase.msg"
    "/home/beom/catkin_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimstatusM2C.msg"
    "/home/beom/catkin_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg/SimStatus.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mujoco_ros_msgs/cmake" TYPE FILE FILES "/home/beom/catkin_ws/build/mujoco_ros_sim/mujoco_ros_msgs/catkin_generated/installspace/mujoco_ros_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/beom/catkin_ws/devel/include/mujoco_ros_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/beom/catkin_ws/devel/share/roseus/ros/mujoco_ros_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/beom/catkin_ws/devel/share/common-lisp/ros/mujoco_ros_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/beom/catkin_ws/devel/share/gennodejs/ros/mujoco_ros_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/beom/catkin_ws/devel/lib/python2.7/dist-packages/mujoco_ros_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/beom/catkin_ws/devel/lib/python2.7/dist-packages/mujoco_ros_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/beom/catkin_ws/build/mujoco_ros_sim/mujoco_ros_msgs/catkin_generated/installspace/mujoco_ros_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mujoco_ros_msgs/cmake" TYPE FILE FILES "/home/beom/catkin_ws/build/mujoco_ros_sim/mujoco_ros_msgs/catkin_generated/installspace/mujoco_ros_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mujoco_ros_msgs/cmake" TYPE FILE FILES
    "/home/beom/catkin_ws/build/mujoco_ros_sim/mujoco_ros_msgs/catkin_generated/installspace/mujoco_ros_msgsConfig.cmake"
    "/home/beom/catkin_ws/build/mujoco_ros_sim/mujoco_ros_msgs/catkin_generated/installspace/mujoco_ros_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mujoco_ros_msgs" TYPE FILE FILES "/home/beom/catkin_ws/src/mujoco_ros_sim/mujoco_ros_msgs/package.xml")
endif()

