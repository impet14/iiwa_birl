# Install script for directory: /home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/birl/worksp/iiwa_stack_birl/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iiwa_msgs/msg" TYPE FILE FILES
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/CartesianControlModeLimits.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/CartesianEulerPose.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/CartesianImpedanceControlMode.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/CartesianPlane.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/CartesianQuantity.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/CartesianVelocity.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/ControlMode.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/DOF.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/DesiredForceControlMode.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/JointDamping.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/JointImpedanceControlMode.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/JointPosition.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/JointPositionVelocity.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/JointQuantity.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/JointStiffness.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/JointTorque.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/JointVelocity.msg"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg/SinePatternControlMode.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iiwa_msgs/srv" TYPE FILE FILES
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/srv/ConfigureSmartServo.srv"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/srv/SetPathParameters.srv"
    "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/srv/TimeToDestination.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iiwa_msgs/cmake" TYPE FILE FILES "/home/birl/worksp/iiwa_stack_birl/build/iiwa_msgs/catkin_generated/installspace/iiwa_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/birl/worksp/iiwa_stack_birl/devel/include/iiwa_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/birl/worksp/iiwa_stack_birl/devel/share/roseus/ros/iiwa_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/birl/worksp/iiwa_stack_birl/devel/share/common-lisp/ros/iiwa_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/birl/worksp/iiwa_stack_birl/devel/share/gennodejs/ros/iiwa_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/birl/worksp/iiwa_stack_birl/devel/lib/python2.7/dist-packages/iiwa_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/birl/worksp/iiwa_stack_birl/devel/lib/python2.7/dist-packages/iiwa_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/birl/worksp/iiwa_stack_birl/build/iiwa_msgs/catkin_generated/installspace/iiwa_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iiwa_msgs/cmake" TYPE FILE FILES "/home/birl/worksp/iiwa_stack_birl/build/iiwa_msgs/catkin_generated/installspace/iiwa_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iiwa_msgs/cmake" TYPE FILE FILES
    "/home/birl/worksp/iiwa_stack_birl/build/iiwa_msgs/catkin_generated/installspace/iiwa_msgsConfig.cmake"
    "/home/birl/worksp/iiwa_stack_birl/build/iiwa_msgs/catkin_generated/installspace/iiwa_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iiwa_msgs" TYPE FILE FILES "/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/package.xml")
endif()

