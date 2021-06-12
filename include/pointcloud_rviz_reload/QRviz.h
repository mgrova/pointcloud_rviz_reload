
#ifndef QRVIZ_H_
#define QRVIZ_H_

#include <QObject>
#include <QColor>
#include <QVBoxLayout>
#include <QDebug>
#include <QVector3D>

#include <ros/ros.h>

namespace rviz
{
class RenderPanel;
class VisualizationManager;
class ViewManager;
class GridDisplay;
class TFDisplay;
class ToolManager;
class Tool;
class Axes;
class Shape;
}

namespace Ogre
{
class Vector3;
class SceneNode;
}

namespace pointcloud_rviz_reload
{
class QRviz : public QObject
{
Q_OBJECT
public:
	QRviz(QObject *parent = nullptr);
	~QRviz();

	void initializeRender();

	QWidget * getMapView();

public Q_SLOTS: 
	void showPointCloudFromPCD(const QString &);

private:
	void setOriginAxis();
	void displayGrid(const QString, const int, const float, const QColor);

private:
	std::unique_ptr<rviz::RenderPanel> _render;
	rviz::VisualizationManager*        _visualizationManager;
	std::unique_ptr<rviz::ViewManager> _viewManager;
	
	std::unique_ptr<rviz::Axes> _originAxe;

	rviz::GridDisplay*   _gridDisplay;

	rviz::ToolManager* _toolManager;
};
	
}

#endif
