
SET(APP_NAME   rviz_viewer)
add_executable(${APP_NAME} main.cpp)
target_link_libraries(${APP_NAME} LINK_PUBLIC ${PROJECT_NAME})
target_include_directories(${APP_NAME} PUBLIC include)
