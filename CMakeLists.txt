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
