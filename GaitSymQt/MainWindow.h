#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileInfo>

#include "QTKitHelper.h"

namespace Ui
{
    class MainWindow;
}

class GLWidget;
class DialogVisibility;
class QBoxLayout;
class QListWidgetItem;
class ViewControlWidget;
class DialogLog;
class Preferences;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    Preferences *GetPreferences() { return preferences; }

public slots:
    void setInterfaceValues();
    void processOneThing();

    void buttonCameraRight();
    void buttonCameraTop();
    void buttonCameraFront();
    void spinboxDistanceChanged(double);
    void spinboxFoVChanged(double);
    void spinboxCOIXChanged(double);
    void spinboxCOIYChanged(double);
    void spinboxCOIZChanged(double);
    void checkboxTracking(int);
    void lineeditTrackingOffset();
    void checkboxOverlay(int);
    void checkboxContactForce(int);
    void checkboxMuscleForce(int);
    void checkboxFramerate(int);
    void checkboxActivationColours(int);
    void checkboxOpenCL(int);
    void lineeditMovieFolder(QString);
    void radioPPM(bool);
    void radioTIFF(bool);
    void radioPOVRay(bool);
    void radioOBJ(bool);
    void radioQuicktime(bool);
    void spinboxSkip(int);
    void spinboxTimeMax(double);
    void listMuscleChecked(QListWidgetItem*);
    void listBodyChecked(QListWidgetItem*);
    void listJointChecked(QListWidgetItem*);
    void listGeomChecked(QListWidgetItem*);
    void menuPreferences();
    void menuOutputs();
    void open();
    void about();
    void snapshot();
    void step();
    void run();
    void restart();
    void saveas();
    void saveasworld();
    void menuDefaultView();
    void menuSaveDefaultView();
    void menuRequestMuscle(QPoint);
    void menuRequestBody(QPoint);
    void menuRequestJoint(QPoint);
    void menuRequestGeom(QPoint);
    void menu640x480();
    void menu800x600();
    void menu1280x720();
    void menu1920x1080();
    void resizeAndCentre(int w, int h);
    void menuStartQuicktimeSave();
    void menuStopQuicktimeSave();
    void menuInterface();
    void newFromGeometry();
    void menuStartWarehouseExport();
    void menuStopWarehouseExport();
    void menuImportWarehouse();
    void menuRecordMovie();
    void menuToggleFullScreen();

    void copy();
    void cut();
    void paste();
    void selectAll();

    void setStatusString(QString s);
    void setUICOI(double x, double y, double z);
    void setUIFoV(double v);

    void log(const char *text);
    //void glwidgetGrabKeyboard();
    //void glwidgetReleaseKeyboard();

protected:
    void keyPressEvent(QKeyEvent *event);
    void closeEvent(QCloseEvent *event);

private:

    void fillVisibitilityLists();
    //void setDefaultLights(bool yUp);
    void updateGL();
    void writeSettings();
    static QByteArray readResource(const QString &resource);

    Ui::MainWindow *ui;

    QTimer *timer;
    int skip;
    long long stepCount;
    bool stepFlag;
    bool trackingFlag;
    double trackingOffset;
    bool allowGLUpdateFlag;

    bool movieFlag;
    QString movieFolder;
    QFileInfo configFile;

    bool newFromGeometryFlag;

    Preferences *preferences;

};

#endif // MAINWINDOW_H
