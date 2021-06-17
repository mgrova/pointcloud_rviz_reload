//---------------------------------------------------------------------------------------------------------------------
//  POINTCLOUD RVIZ RELOAD APP
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2021 Marco A. Montes Grova (a.k.a. mgrova) marrcogrova@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

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
