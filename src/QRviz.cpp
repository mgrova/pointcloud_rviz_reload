
#include "pointcloud_rviz_reload/QRviz.h"
#include "pointcloud_rviz_reload/PCDFileHandler.h"

#include <rviz/display_group.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <rviz/default_plugin/grid_display.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/default_plugin/tf_display.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/point_cloud.h>

#include <chrono>

namespace pointcloud_rviz_reload
{

QRviz::QRviz(QObject *parent) : QObject(parent)
, _render(std::make_unique<rviz::RenderPanel>())
{
    
}

QRviz::~QRviz()
{
    _visualizationManager->removeAllDisplays();
    _toolManager->removeAll();
}

QWidget * QRviz::getMapView()
{
    return dynamic_cast<QWidget*>(_render.get());
}

void QRviz::initializeRender()
{
    _visualizationManager = new rviz::VisualizationManager(_render.get());

    _render->initialize(_visualizationManager->getSceneManager(), _visualizationManager);
    _visualizationManager->initialize();
    _visualizationManager->removeAllDisplays();
    _visualizationManager->setFixedFrame("map");

    _viewManager = std::unique_ptr<rviz::ViewManager>(_visualizationManager->getViewManager());
    _viewManager->setRenderPanel(_render.get());
    _viewManager->setCurrentViewControllerType("rviz/XYOrbit");

    _toolManager = _visualizationManager->getToolManager();
    _toolManager->initialize();

    displayGrid("map", 16, 1.0f, QColor(125,125,125));
    setOriginAxis();

    _visualizationManager->startUpdate();
}

void QRviz::setOriginAxis()
{
    _originAxe = std::make_unique<rviz::Axes>(_visualizationManager->getSceneManager(), nullptr, 0.4f, 0.05f);
    _originAxe->setPosition(Ogre::Vector3(0.0, 0.0, 0.0));
    _originAxe->setOrientation(Ogre::Quaternion(1.0, 0.0, 0.0, 0.0));
    _originAxe->getSceneNode()->setVisible(true);
}

void QRviz::displayGrid(const QString referenceFrame, const int planCellCount, const float metersPerCell, const QColor color)
{
    _gridDisplay = dynamic_cast<rviz::GridDisplay*>(_visualizationManager->createDisplay( "rviz/Grid", "QGrid", true));
    ROS_ASSERT( _gridDisplay != nullptr );

    _gridDisplay->subProp("Reference Frame")->setValue(referenceFrame);
    _gridDisplay->subProp("Plane Cell Count")->setValue(planCellCount);
    _gridDisplay->subProp("Cell Size")->setValue(metersPerCell);
    _gridDisplay->subProp("Color")->setValue(color);
    _gridDisplay->subProp("Line Style")->setValue("Billboards");

    _gridDisplay->setEnabled(true);
}

void QRviz::showPointCloudFromPCD(const QString &pcd_path)
{
    auto parser = std::make_unique<PCDFileHandler>();
    rviz::PointCloud* tgt_cloud = new rviz::PointCloud();
    
    std::cout << "[showPointCloudFromPCD] Reading pointcloud from: " << pcd_path.toStdString() << std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    tgt_cloud = parser->convertFileToPointCloud(pcd_path);
    
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "[showPointCloudFromPCD] Elapsed time loading cloud: " 
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0f << " [s]" << std::endl;

    Ogre::SceneNode* scene_node = _visualizationManager->getSceneManager()->getRootSceneNode()->createChildSceneNode();           
    scene_node->attachObject(tgt_cloud);
}


} // namespace gcs
