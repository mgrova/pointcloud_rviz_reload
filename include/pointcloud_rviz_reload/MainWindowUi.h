
#ifndef MAIN_WINDOW_UI_H_
#define MAIN_WINDOW_UI_H_

#include <QMainWindow>
#include <memory> 
#include <thread>

namespace pointcloud_rviz_reload
{
class QRviz;

class MainWindowUi : public QMainWindow
{
Q_OBJECT
public:
    MainWindowUi();
    ~MainWindowUi();

Q_SIGNALS:
    void showPointCloudFromPCD(const QString &);

private:
    void createMenu();

private:
    std::unique_ptr<QRviz> _viewer;
};
}

#endif