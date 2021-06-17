##---------------------------------------------------------------------------------------------------------------------
##  POINTCLOUD RVIZ RELOAD APP
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2021 Marco A. Montes Grova (a.k.a. mgrova) marrcogrova@gmail.com
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
PROJECT(pointcloud_rviz_reload VERSION 0.1.0)

#########################################
######			General config	   ######
#########################################
set(CMAKE_AUTOMOC ON)
option(BUILD_TOOLS "Build tools" ON)

##################################################
######    Library.                        ########
##################################################
file(GLOB_RECURSE PRR_HEADERS "./include/*.h")
file(GLOB_RECURSE PRR_SOURCES "./src/*.cpp")

add_library(${PROJECT_NAME} SHARED ${PRR_HEADERS} ${PRR_SOURCES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

##################################################
######    Loading 3rd party libraries.    ########
##################################################
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)    
if(${Qt5_FOUND})
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets)
endif(${Qt5_FOUND})

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC pthread)

find_package(catkin REQUIRED COMPONENTS rviz roscpp pcl_ros)
if(${catkin_FOUND})
    catkin_package(
        INCLUDE_DIRS   include
        LIBRARIES      ${PROJECT_NAME}
        CATKIN_DEPENDS rviz roscpp pcl_ros
        DEPENDS        )
    
    target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
endif(${catkin_FOUND})

##################################################
######    Tools.                          ########
##################################################
if(${BUILD_TOOLS})
    add_subdirectory(tools)
endif(${BUILD_TOOLS})
