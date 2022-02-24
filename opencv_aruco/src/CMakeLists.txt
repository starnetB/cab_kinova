#寻找OpenCV库
FIND_PACKAGE(OpenCV REQUIRED)
#SET(MVIG_REAL /home/ziye01/realsence/opencv_aruco/install/lib)
#find_package(realscene_cam_opt REQUIRED )
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${realsense2_INCLUDE_DIRS})

#打印调试信息
MESSAGE(STATUS "Project: ${PROJECT_NAME}")
MESSAGE(STATUS "OpenCV library status:")
MESSAGE(STATUS "    version: ${OpenCV_VERSION}")
MESSAGE(STATUS "    libraries: ${OpenCV_LIBS}")
MESSAGE(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()
#获取代码，在项目中，将所有代码都放在src文件夹中
#AUX_SOURCE_DIRECTORY(src DIR_SRCS)
#MESSAGE(STATUS "Src file: ${DIR_SRCS}")

# 以下部分用于编译动态库  
# 安装前缀
SET(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)    
SET(INCLUDESRC ../include)
SET(LIBSRC realscene_cam_opt.cpp )
INCLUDE_DIRECTORIES(${INCLUDESRC})
add_library(realscene_cam_opt SHARED ${LIBSRC} ${realsense2_LIBS})  

#编译可执行程序
#ADD_EXECUTABLE(${PROJECT_NAME} ${DIR_SRCS})
ADD_EXECUTABLE(createMarker createMarker.cpp)
ADD_EXECUTABLE(Intrintic realsenceIntrintics.cpp)

#添加链接库
TARGET_LINK_LIBRARIES(createMarker ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(Intrintic ${OpenCV_LIBS} realscene_cam_opt ${REALSENSE2_FOUND})


# 安装到安装路径
INSTALL(FILES  ${INCLUDESRC}/realscene_cam_opt.hpp DESTINATION ${CMAKE_INSTALL_PREFIX}/include )                   
INSTALL(TARGETS realscene_cam_opt LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
INSTALL(TARGETS createMarker 
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
INSTALL(TARGETS Intrintic  
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
        