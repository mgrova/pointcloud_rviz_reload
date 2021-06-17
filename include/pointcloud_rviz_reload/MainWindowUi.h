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