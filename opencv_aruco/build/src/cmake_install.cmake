# Install script for directory: /home/ziye01/realsence/opencv_aruco/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ziye01/realsence/opencv_aruco/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ziye01/realsence/opencv_aruco/install/include/realscene_cam_opt.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ziye01/realsence/opencv_aruco/install/include" TYPE FILE FILES "/home/ziye01/realsence/opencv_aruco/src/../include/realscene_cam_opt.hpp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/lib/librealscene_cam_opt.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/lib/librealscene_cam_opt.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/lib/librealscene_cam_opt.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ziye01/realsence/opencv_aruco/install/lib/librealscene_cam_opt.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ziye01/realsence/opencv_aruco/install/lib" TYPE SHARED_LIBRARY FILES "/home/ziye01/realsence/opencv_aruco/build/src/librealscene_cam_opt.so")
  if(EXISTS "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/lib/librealscene_cam_opt.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/lib/librealscene_cam_opt.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/lib/librealscene_cam_opt.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/createMarker" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/createMarker")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/createMarker"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ziye01/realsence/opencv_aruco/install/bin/createMarker")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ziye01/realsence/opencv_aruco/install/bin" TYPE EXECUTABLE FILES "/home/ziye01/realsence/opencv_aruco/build/src/createMarker")
  if(EXISTS "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/createMarker" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/createMarker")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/createMarker")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/Intrintic" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/Intrintic")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/Intrintic"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ziye01/realsence/opencv_aruco/install/bin/Intrintic")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ziye01/realsence/opencv_aruco/install/bin" TYPE EXECUTABLE FILES "/home/ziye01/realsence/opencv_aruco/build/src/Intrintic")
  if(EXISTS "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/Intrintic" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/Intrintic")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/Intrintic"
         OLD_RPATH "/home/ziye01/realsence/opencv_aruco/build/src:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/ziye01/realsence/opencv_aruco/install/bin/Intrintic")
    endif()
  endif()
endif()

