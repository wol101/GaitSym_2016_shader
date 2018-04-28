#include <QApplication>
#include <QCoreApplication>
#include <QSettings>
#include <QSurfaceFormat>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QSurfaceFormat fmt;
    fmt.setRenderableType(QSurfaceFormat::OpenGL);
    fmt.setDepthBufferSize(24);
    fmt.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    fmt.setSamples(1);
    fmt.setVersion(4, 1); // OpenGL 4.1
    fmt.setProfile(QSurfaceFormat::CoreProfile); // only use the core functions
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication a(argc, argv);

    MainWindow w;
    w.show();
    return a.exec();
}
