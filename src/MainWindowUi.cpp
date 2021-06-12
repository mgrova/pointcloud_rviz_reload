
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
