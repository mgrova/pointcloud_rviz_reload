
#include <QApplication>
#include <memory>
#include <ros/ros.h>
#include "pointcloud_rviz_reload/MainWindowUi.h"
#include "pointcloud_rviz_reload/PCDFileHandler.h"

int main(int argc, char **argv)
{    
    ros::init(argc, argv, "viewer_node");
    ROS_INFO("[%s] Node initialization.", ros::this_node::getName().data());
    ros::AsyncSpinner spinner(4); // We use 4 thread to spin
    spinner.start();

    QApplication app(argc, argv);
    QCoreApplication::setApplicationName("RViz Viewer");
    auto window = std::make_unique<pointcloud_rviz_reload::MainWindowUi>();
    window->show();

    return app.exec();
}