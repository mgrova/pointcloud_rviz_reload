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

#include "pointcloud_rviz_reload/MainWindowUi.h"
#include "pointcloud_rviz_reload/QRviz.h"

#include <QStatusBar>
#include <QMenuBar>
#include <QToolBar>
#include <QFileDialog>

namespace pointcloud_rviz_reload
{
MainWindowUi::MainWindowUi() : _viewer(std::make_unique<QRviz>())
{
    _viewer->initializeRender();
    setCentralWidget(_viewer->getMapView());
    createMenu();
    statusBar()->showMessage(tr("Ready"));

    connect(this,          &MainWindowUi::showPointCloudFromPCD,
            _viewer.get(), &QRviz::showPointCloudFromPCD);
}

MainWindowUi::~MainWindowUi()
{
  
}

void MainWindowUi::createMenu()
{
    QMenu *fileMenu = menuBar()->addMenu(tr("&Menu"));
    
    QAction *selectionAct = new QAction(tr("&Select PCD File"), this);
    selectionAct->setStatusTip(tr("Select PCD file to load the cloud"));
    connect(selectionAct, &QAction::triggered, [=]() 
        { 
            auto pcdFilePath = QFileDialog::getOpenFileName(
                this, tr("Open PCD File"), QDir::homePath(),
                tr("PCD PointCloud (*.pcd)"), nullptr, QFileDialog::DontUseNativeDialog);
            if(pcdFilePath.isEmpty())
                return;

            Q_EMIT showPointCloudFromPCD(pcdFilePath);
            statusBar()->showMessage(tr("Showing file: ") + pcdFilePath); 
        });
    
    fileMenu->addAction(selectionAct);
}
}
