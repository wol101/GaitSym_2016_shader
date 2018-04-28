#include <QMessageBox>
#include <QTimer>
#include <QFileDialog>
#include <QBoxLayout>
#include <QDesktopWidget>
#include <QListWidgetItem>
#include <QSettings>
#include <QLineEdit>
#include <QFile>
#include <QKeyEvent>
#include <QRegExp>
#include <QDir>
#include <QStringList>
#include <QTemporaryFile>
#include <QSizePolicy>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "dialogpreferences.h"
#include "ui_dialogpreferences.h"
#include "dialoginterface.h"
#include "ui_dialoginterface.h"
#include "dialogoutputselect.h"
#include "ui_dialogoutputselect.h"
#include "aboutdialog.h"
#include "customfiledialogs.h"
#include "glwidget.h"
#include "viewcontrolwidget.h"
#include "ObjectiveMain.h"
#include "Simulation.h"
#include "DataFile.h"
#include "Muscle.h"
#include "Body.h"
#include "Joint.h"
#include "Geom.h"
#include "Driver.h"
#include "DataTarget.h"
#include "FacetedObject.h"
#include "Reporter.h"
#include "Warehouse.h"
#include "preferences.h"

#ifdef USE_OPENCL
#include "OpenCLRoutines.h"
#endif

// Simulation global
extern Simulation *gSimulation;

// External display globals
extern int gDrawMuscleForces;
extern int gDrawContactForces;
extern int gActivationDisplay;
extern int gElasticDisplay;
extern int g_UseOpenCL;

// external file paths
extern char *gConfigFilenamePtr;
extern char *gGraphicsRoot;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    // initialise some class variables
    skip = 1;
    stepCount = 0;
    stepFlag = false;
    movieFlag = false;
    trackingFlag = false;
    allowGLUpdateFlag = true;
    trackingOffset = 0;
    newFromGeometryFlag = false;

    // read in the preferences file
    preferences = new Preferences();
    preferences->Read();

    // create the window elements
    ui->setupUi(this);

    // interface related connections
    connect(ui->action_NewFromGeometry, SIGNAL(triggered()), this, SLOT(newFromGeometry()));
    connect(ui->action_Open, SIGNAL(triggered()), this, SLOT(open()));
    connect(ui->action_Quit, SIGNAL(triggered()), this, SLOT(close()));
    connect(ui->action_Restart, SIGNAL(triggered()), this, SLOT(restart()));
    connect(ui->action_Save, SIGNAL(triggered()), this, SLOT(saveas()));
    connect(ui->action1280x720, SIGNAL(triggered()), this, SLOT(menu1280x720()));
    connect(ui->action1920x1080, SIGNAL(triggered()), this, SLOT(menu1920x1080()));
    connect(ui->action640x480, SIGNAL(triggered()), this, SLOT(menu640x480()));
    connect(ui->action800x600, SIGNAL(triggered()), this, SLOT(menu800x600()));
    connect(ui->actionAbout_GaitSymQt, SIGNAL(triggered()), this, SLOT(about()));
    connect(ui->actionCopy, SIGNAL(triggered()), this, SLOT(copy()));
    connect(ui->actionCut, SIGNAL(triggered()), this, SLOT(cut()));
    connect(ui->actionDefault_View, SIGNAL(triggered()), this, SLOT(menuDefaultView()));
    connect(ui->actionImport_Warehouse, SIGNAL(triggered()), this, SLOT(menuImportWarehouse()));
    connect(ui->actionInterface, SIGNAL(triggered()), this, SLOT(menuInterface()));
    connect(ui->actionOutput, SIGNAL(triggered()), this, SLOT(menuOutputs()));
    connect(ui->actionPaste, SIGNAL(triggered()), this, SLOT(paste()));
    connect(ui->actionPreferences, SIGNAL(triggered()), this, SLOT(menuPreferences()));
    connect(ui->actionRecord_Movie, SIGNAL(triggered()), this, SLOT(menuRecordMovie()));
    connect(ui->actionSave_as_World, SIGNAL(triggered()), this, SLOT(saveasworld()));
    connect(ui->actionSave_Default_View, SIGNAL(triggered()), this, SLOT(menuSaveDefaultView()));
    connect(ui->actionToggleFullscreen, SIGNAL(triggered()), this, SLOT(menuToggleFullScreen()));
    connect(ui->actionSelect_all, SIGNAL(triggered()), this, SLOT(selectAll()));
    connect(ui->actionStart_Warehouse_Export, SIGNAL(triggered()), this, SLOT(menuStartWarehouseExport()));
    connect(ui->actionStop_Warehouse_Export, SIGNAL(triggered()), this, SLOT(menuStopWarehouseExport()));
    connect(ui->checkBoxActivationColours, SIGNAL(stateChanged(int)), this, SLOT(checkboxActivationColours(int)));
    connect(ui->checkBoxContactForce, SIGNAL(stateChanged(int)), this, SLOT(checkboxContactForce(int)));
    connect(ui->checkBoxFramerate, SIGNAL(stateChanged(int)), this, SLOT(checkboxFramerate(int)));
    connect(ui->checkBoxMuscleForce, SIGNAL(stateChanged(int)), this, SLOT(checkboxMuscleForce(int)));
    connect(ui->checkBoxOverlay, SIGNAL(stateChanged(int)), this, SLOT(checkboxOverlay(int)));
    connect(ui->checkBoxTracking, SIGNAL(stateChanged(int)), this, SLOT(checkboxTracking(int)));
    connect(ui->doubleSpinBoxCOIX, SIGNAL(valueChanged(double)), this, SLOT(spinboxCOIXChanged(double)));
    connect(ui->doubleSpinBoxCOIY, SIGNAL(valueChanged(double)), this, SLOT(spinboxCOIYChanged(double)));
    connect(ui->doubleSpinBoxCOIZ, SIGNAL(valueChanged(double)), this, SLOT(spinboxCOIZChanged(double)));
    connect(ui->doubleSpinBoxDistance, SIGNAL(valueChanged(double)), this, SLOT(spinboxDistanceChanged(double)));
    connect(ui->doubleSpinBoxFoV, SIGNAL(valueChanged(double)), this, SLOT(spinboxFoVChanged(double)));
    connect(ui->doubleSpinBoxTimeMax, SIGNAL(valueChanged(double)), this, SLOT(spinboxTimeMax(double)));
    connect(ui->lineEditMovieFolder, SIGNAL(textChanged(QString)), this, SLOT(lineeditMovieFolder(QString)));
    connect(ui->lineEditTrackingOffset, SIGNAL(editingFinished()), this, SLOT(lineeditTrackingOffset()));
    connect(ui->listWidgetBody, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(menuRequestBody(QPoint)));
    connect(ui->listWidgetBody, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(listBodyChecked(QListWidgetItem*)));
    connect(ui->listWidgetGeom, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(menuRequestGeom(QPoint)));
    connect(ui->listWidgetGeom, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(listGeomChecked(QListWidgetItem*)));
    connect(ui->listWidgetJoint, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(menuRequestJoint(QPoint)));
    connect(ui->listWidgetJoint, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(listJointChecked(QListWidgetItem*)));
    connect(ui->listWidgetMuscle, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(menuRequestMuscle(QPoint)));
    connect(ui->listWidgetMuscle, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(listMuscleChecked(QListWidgetItem*)));
    //connect(ui->pushButtonFront, SIGNAL(clicked()), this, SLOT(buttonCameraFront()));
    //connect(ui->pushButtonRight, SIGNAL(clicked()), this, SLOT(buttonCameraRight()));
    //connect(ui->pushButtonTop, SIGNAL(clicked()), this, SLOT(buttonCameraTop()));
    connect(ui->actionViewFront, SIGNAL(triggered()), this, SLOT(buttonCameraFront()));
    connect(ui->actionViewRight, SIGNAL(triggered()), this, SLOT(buttonCameraRight()));
    connect(ui->actionViewTop, SIGNAL(triggered()), this, SLOT(buttonCameraTop()));
    connect(ui->radioButtonOBJ, SIGNAL(clicked(bool)), this, SLOT(radioOBJ(bool)));
    connect(ui->radioButtonPOVRay, SIGNAL(clicked(bool)), this, SLOT(radioPOVRay(bool)));
    connect(ui->radioButtonPPM, SIGNAL(clicked(bool)), this, SLOT(radioPPM(bool)));
    connect(ui->radioButtonQuicktime, SIGNAL(clicked(bool)), this, SLOT(radioQuicktime(bool)));
    connect(ui->radioButtonTIFF, SIGNAL(clicked(bool)), this, SLOT(radioTIFF(bool)));
    connect(ui->spinBoxSkip, SIGNAL(valueChanged(int)), this, SLOT(spinboxSkip(int)));
    //connect(ui->toolButtonPlay, SIGNAL(clicked()), this, SLOT(run()));
    //connect(ui->toolButtonSnapshot, SIGNAL(clicked()), this, SLOT(snapshot()));
    //connect(ui->toolButtonStep, SIGNAL(clicked()), this, SLOT(step()));
    connect(ui->actionRun, SIGNAL(triggered()), this, SLOT(run()));
    connect(ui->actionSnapshot, SIGNAL(triggered()), this, SLOT(snapshot()));
    connect(ui->actionStep, SIGNAL(triggered()), this, SLOT(step()));

#ifdef OLD_INTERFACE
    // position the elements

    QDesktopWidget *desktop = qApp->desktop();
    QRect available = desktop->availableGeometry(-1);

    // careful about the various Qt size functions
    // Including the window frame:
    // x(), y(), frameGeometry(), pos() and move()
    // Excluding the window frame:
    // geometry(), width(), height(), rect() and size()


#ifdef WINDOWS_NOT_OVERLAPPED
    ui->dockWidgetControls->move(available.left(), available.top());
    ui->dockWidgetVisibility->move(available.right() - ui->dockWidgetVisibility->frameGeometry().width(), available.top());

    move(available.left() + ui->dockWidgetControls->frameGeometry().width(), available.top());
    resize(available.width() - (ui->dockWidgetControls->frameGeometry().width() + ui->dockWidgetVisibility->frameGeometry().width()),
                 available.height() - (frameGeometry().height() - height()));
#else
    move(available.left(), available.top());
    resize(available.width() - (frameGeometry().width() - width()),
                 available.height() - (frameGeometry().height() - height()));

#ifdef __APPLE__
    int verticalSpace = (frameGeometry().height() - height());
#else
    int verticalSpace = (frameGeometry().height() - height() + ui->menuBar->height());
#endif
    ui->dockWidgetControls->setFixedSize(ui->dockWidgetControls->frameGeometry().width(),ui->dockWidgetControls->frameGeometry().height() - 30);
    ui->dockWidgetControls->move(available.left(), available.top() + verticalSpace);
    ui->dockWidgetVisibility->setFixedSize(ui->dockWidgetVisibility->frameGeometry().width(),ui->dockWidgetVisibility->frameGeometry().height() - 30);
    ui->dockWidgetVisibility->move(available.right() - ui->dockWidgetVisibility->frameGeometry().width(), available.top() + verticalSpace);
    ui->dockWidgetView->setFixedSize(ui->dockWidgetView->frameGeometry().width(),ui->dockWidgetView->frameGeometry().height() - 30);
    ui->dockWidgetView->move(available.right() - ui->dockWidgetView->frameGeometry().width(), available.top() + verticalSpace + ui->dockWidgetVisibility->frameGeometry().height());

    ui->dockWidgetLog->move(available.right() - ui->dockWidgetLog->frameGeometry().width(),
                            available.top() + verticalSpace + ui->dockWidgetVisibility->frameGeometry().height() + ui->dockWidgetView->frameGeometry().height());

#endif

    // put GLWidget into centralWidget
    boxLayout = new QBoxLayout(QBoxLayout::LeftToRight, ui->centralWidget);
    boxLayout->setMargin(0);
    glWidget = new GLWidget();
    QSurfaceFormat fmt;
    fmt.setSamples(preferences->OpenGLMultisample);
    glWidget->setFormat(fmt);
    boxLayout->addWidget(glWidget);
    glWidget->grabKeyboard();

    // put ViewControlWidget into widgetViewFrame
    boxLayout2 = new QBoxLayout(QBoxLayout::LeftToRight, ui->widgetViewFrame);
    boxLayout2->setMargin(0);
    viewControlWidget = new ViewControlWidget();
    boxLayout2->addWidget(viewControlWidget);

    // connect the ViewControlWidget to the GLWidget
    QObject::connect(viewControlWidget, SIGNAL(EmitCameraVec(double, double, double)), glWidget, SLOT(SetCameraVec(double, double, double)));

    // connect the GLWidget to the MainWindow
    QObject::connect(glWidget, SIGNAL(EmitStatusString(QString)), this, SLOT(setStatusString(QString)));
    QObject::connect(glWidget, SIGNAL(EmitCOI(double, double, double)), this, SLOT(setUICOI(double, double, double)));
    QObject::connect(glWidget, SIGNAL(EmitFoV(double)), this, SLOT(setUIFoV(double)));
#endif

    // connect the ViewControlWidget to the GLWidget
    QObject::connect(ui->widgetViewFrame, SIGNAL(EmitCameraVec(double, double, double)), ui->widgetGLWidget, SLOT(SetCameraVec(double, double, double)));

    // connect the GLWidget to the MainWindow
    QObject::connect(ui->widgetGLWidget, SIGNAL(EmitStatusString(QString)), this, SLOT(setStatusString(QString)));
    QObject::connect(ui->widgetGLWidget, SIGNAL(EmitCOI(double, double, double)), this, SLOT(setUICOI(double, double, double)));
    QObject::connect(ui->widgetGLWidget, SIGNAL(EmitFoV(double)), this, SLOT(setUIFoV(double)));

    // set up the timer
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(processOneThing()));

    // zero the timer
    QString time = QString("%1").arg(double(0), 0, 'f', 5);
    ui->lcdNumberTime->display(time);

    setInterfaceValues();
    menuDefaultView();

    statusBar()->showMessage(tr("Ready"));

    // set menu activations for no loaded model
    ui->actionOutput->setEnabled(false);
    ui->action_Restart->setEnabled(false);
    ui->action_Save->setEnabled(false);
    ui->actionSave_as_World->setEnabled(false);
    ui->actionStart_Warehouse_Export->setEnabled(false);
    ui->actionStop_Warehouse_Export->setEnabled(false);
    ui->actionImport_Warehouse->setEnabled(false);
    ui->actionRecord_Movie->setEnabled(false);
    ui->actionRun->setEnabled(false);
    ui->actionStep->setEnabled(false);
    ui->actionSnapshot->setEnabled(false);

#ifndef __APPLE__
    // quicktime only available for Mac version
    ui->radioButtonQuicktime->setVisible(false);
#endif

#ifdef USE_OPENCL
    try
    {
        OpenCLRoutines::InitCL();
    }
    catch (...)
    {
        statusBar()->showMessage(tr("Unable to initialise OpenCL - disabled"));
        ui->checkBoxOpenCL->setChecked(false);
        ui->checkBoxOpenCL->setDisabled(true);
        g_UseOpenCL = false;
    }
#else
    g_UseOpenCL = false;
#endif

#ifndef EXPERIMENTAL
    ui->action640x480->setVisible(false);
    ui->action800x600->setVisible(false);
    ui->action1280x720->setVisible(false);
    ui->action1920x1080->setVisible(false);
    ui->actionStart_Warehouse_Export->setVisible(false);
    ui->actionStop_Warehouse_Export->setVisible(false);
    ui->actionImport_Warehouse->setVisible(false);
#endif

    setUnifiedTitleAndToolBarOnMac(false);

    // finally remember the geometry etc.
    if (preferences->MainWindowGeometry.size() > 0) restoreGeometry(preferences->MainWindowGeometry);
    if (preferences->MainWindowState.size() > 0) restoreState(preferences->MainWindowState);
    if (preferences->SplitterState.size() > 0) ui->splitter->restoreState(preferences->SplitterState);
}

MainWindow::~MainWindow()
{
#ifdef USE_OPENCL
    try
    {
        OpenCLRoutines::ReleaseCL();
    }
    catch (...)
    {
        std::cerr << "Error releasing OpenCL\n";
    }
#endif

    timer->stop();

    if (gSimulation) delete gSimulation;
    if (ui->widgetGLWidget->GetQTMovie()) closeMovieFile(ui->widgetGLWidget->GetQTMovie());
    //delete timer;
    //delete glWidget;
    //delete boxLayout;
    //delete viewControlWidget;
    //delete boxLayout2;
    delete preferences;
    delete ui;

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    writeSettings();

    QMainWindow::closeEvent(event);
}

// handle key presses
void MainWindow::keyPressEvent( QKeyEvent *e )
{
    switch( e->key() )
    {
    // X, Y, Z & Shandled by GLWindow
    case Qt::Key_X:
    case Qt::Key_Y:
    case Qt::Key_Z:
    case Qt::Key_S:
        ui->widgetGLWidget->HandleKeyPressEvent(e);
        break;

    default:
        QMainWindow::keyPressEvent( e );
    }
}

void MainWindow::newFromGeometry()
{
    newFromGeometryFlag = true;
    open();
    newFromGeometryFlag = false;
}


void MainWindow::open()
{
    QFileInfo info = preferences->LastFileOpened;
    while (info.exists() == false)
    {
        info = QFileInfo(info.dir().absolutePath());
    }

    QString fileName;
    QStringList geometryFileNames;
    QTemporaryFile *tempFile = 0;

    if (newFromGeometryFlag)
    {
        geometryFileNames = CustomFileDialogs::getOpenFileNames(this, tr("Select the geometry files required"), info.absoluteFilePath(), tr("Geometry Files (*.obj)"), 0, QFileDialog::DontUseNativeDialog);
    }
    else
    {
        fileName = CustomFileDialogs::getOpenFileName(this, tr("Open Config File"), info.absoluteFilePath(), tr("Config Files (*.xml)"), 0, QFileDialog::DontUseNativeDialog);
    }

    if (fileName.isNull() == false || geometryFileNames.size() > 0)
    {
        if (gSimulation) delete gSimulation;
        gSimulation = 0;
        stepCount = 0;

        ui->actionRun->setChecked(false);
        run();

        if (newFromGeometryFlag)
        {
            QFileInfo info(geometryFileNames[0]);
            preferences->GraphicsPath = info.absolutePath();
            preferences->LastFileOpened =  QDir(info.absolutePath()).filePath(info.completeBaseName() + ".xml");
            QString fileTemplate(readResource(":/templates/NewConfigFileTemplate.xml"));
            QString bodyTemplate(readResource(":/templates/BodyTemplate.xml"));
            QString bodyText;
            for (int i = 0; i < geometryFileNames.size(); i++)
            {
                info.setFile(geometryFileNames[i]);
                bodyText.append(bodyTemplate);
                bodyText.replace("ID_BOOKMARK", QString("Body%1").arg(i));
                bodyText.replace("GRAPHICFILE_BOOKMARK", info.fileName());
            }
            fileTemplate.replace("BODY_BOOKMARK", bodyText);
            tempFile = new QTemporaryFile();
            tempFile->open();
            tempFile->write(fileTemplate.toUtf8());
            tempFile->close();
            fileName = tempFile->fileName();
        }
        else
        {
            preferences->LastFileOpened = fileName;
        }
        char *readFileName = new char[fileName.toUtf8().length() + 1];
        strcpy(readFileName, fileName.toUtf8());
        gConfigFilenamePtr = readFileName;

        QString graphicsPath = preferences->GraphicsPath;
        char *graphicsRoot = new char[graphicsPath.toUtf8().length() + 1];
        strcpy(graphicsRoot, graphicsPath.toUtf8());
        gGraphicsRoot = graphicsRoot;

        configFile.setFile(fileName);
        QDir::setCurrent(configFile.absolutePath());

        int err = ReadModel(this);
        if (tempFile) delete tempFile;
        delete [] graphicsRoot;
        delete [] readFileName;
        if (err)
        {
            statusBar()->showMessage(QString("Error loading ") + fileName);
            return;
        }

        fillVisibitilityLists();
        ui->doubleSpinBoxTimeMax->setValue(gSimulation->GetTimeLimit());
        QString time = QString("%1").arg(double(0), 0, 'f', 5);
        ui->lcdNumberTime->display(time);
        checkboxTracking(trackingFlag);

        //setDefaultLights(preferences->YUp);
        updateGL();
        statusBar()->showMessage(fileName + QString(" loaded"));

        // put the filename as a title
        if (fileName.size() <= 256) setWindowTitle(fileName);
        else setWindowTitle(QString("...") + fileName.right(256));

        // set menu activations for loaded model
        ui->actionOutput->setEnabled(true);
        ui->action_Restart->setEnabled(true);
        ui->action_Save->setEnabled(true);
        ui->actionSave_as_World->setEnabled(true);
        ui->actionStart_Warehouse_Export->setEnabled(true);
        ui->actionStop_Warehouse_Export->setEnabled(false);
        ui->actionImport_Warehouse->setEnabled(true);
        ui->actionRecord_Movie->setEnabled(true);
        ui->actionRun->setEnabled(true);
        ui->actionStep->setEnabled(true);
        ui->actionSnapshot->setEnabled(true);
    }
}

void MainWindow::restart()
{
    if (gSimulation == 0) return;
    delete gSimulation;
    gSimulation = 0;
    stepCount = 0;

    char *readFileName = new char[configFile.absoluteFilePath().toUtf8().length() + 1];
    strcpy(readFileName, configFile.absoluteFilePath().toUtf8());
    gConfigFilenamePtr = readFileName;

    QString graphicsPath = preferences->GraphicsPath;
    char *graphicsRoot = new char[graphicsPath.toUtf8().length() + 1];
    strcpy(graphicsRoot, graphicsPath.toUtf8());
    gGraphicsRoot = graphicsRoot;

    ReadModel(this);
    delete [] graphicsRoot;
    delete [] readFileName;

    fillVisibitilityLists();
    ui->doubleSpinBoxTimeMax->setValue(gSimulation->GetTimeLimit());
    QString time = QString("%1").arg(double(0), 0, 'f', 5);
    ui->lcdNumberTime->display(time);
    checkboxTracking(trackingFlag);
    updateGL();
    statusBar()->showMessage(configFile.fileName() + QString(" reloaded"));

    // set menu activations for loaded model
    ui->actionOutput->setEnabled(true);
    ui->action_Restart->setEnabled(true);
    ui->action_Save->setEnabled(true);
    ui->actionSave_as_World->setEnabled(true);
    ui->actionStart_Warehouse_Export->setEnabled(true);
    ui->actionStop_Warehouse_Export->setEnabled(false);
    ui->actionImport_Warehouse->setEnabled(true);
}

void MainWindow::saveas()
{
    QFileInfo info = preferences->LastFileOpened;

    QString fileName = CustomFileDialogs::getSaveFileName(this, tr("Save Model State File (Relative)"), info.absolutePath(), tr("Config Files (*.xml)"), 0, QFileDialog::DontUseNativeDialog);

    if (fileName.isNull() == false)
    {
        gSimulation->SetModelStateRelative(true);
        gSimulation->SetOutputModelStateFile(fileName.toUtf8());
        gSimulation->OutputProgramState();
    }
}

void MainWindow::saveasworld()
{
    QFileInfo info = preferences->LastFileOpened;

    QString fileName = CustomFileDialogs::getSaveFileName(this, tr("Save Model State File (World)"), info.absolutePath(), tr("Config Files (*.xml)"), 0, QFileDialog::DontUseNativeDialog);

    if (fileName.isNull() == false)
    {
        gSimulation->SetModelStateRelative(false);
        gSimulation->SetOutputModelStateFile(fileName.toUtf8());
        gSimulation->OutputProgramState();
    }
}


void MainWindow::about()
 {
    AboutDialog aboutDialog(this);

    int status = aboutDialog.exec();

    if (status == QDialog::Accepted)
    {
    }
 }

void MainWindow::run()
{
    if (ui->actionRun->isChecked())
    {
        if (gSimulation) timer->start();
        statusBar()->showMessage(tr("Simulation running"));
    }
    else
    {
        timer->stop();
        statusBar()->showMessage(tr("Simulation stopped"));
    }
}

void MainWindow::menuRecordMovie()
{
    if (ui->actionRecord_Movie->isChecked())
    {
        movieFlag = true;
        if (ui->widgetGLWidget->GetMovieFormat() == GLWidget::Quicktime && ui->widgetGLWidget->GetQTMovie() == 0) menuStartQuicktimeSave();
        ui->radioButtonOBJ->setEnabled(false);
        ui->radioButtonPOVRay->setEnabled(false);
        ui->radioButtonPPM->setEnabled(false);
        ui->radioButtonQuicktime->setEnabled(false);
        ui->radioButtonTIFF->setEnabled(false);
    }
    else
    {
        movieFlag = false;
        if (ui->widgetGLWidget->GetMovieFormat() == GLWidget::Quicktime) menuStopQuicktimeSave();
        ui->radioButtonOBJ->setEnabled(true);
        ui->radioButtonPOVRay->setEnabled(true);
        ui->radioButtonPPM->setEnabled(true);
        ui->radioButtonQuicktime->setEnabled(true);
        ui->radioButtonTIFF->setEnabled(true);
    }
}

void MainWindow::step()
{
    stepFlag = true;
    if (gSimulation) timer->start();
    statusBar()->showMessage(tr("Simulation stepped"));
}

void MainWindow::processOneThing()
{
    if (gSimulation)
    {
        if (gSimulation->ShouldQuit() || gSimulation->TestForCatastrophy())
        {
            statusBar()->showMessage(tr("Unable to start simulation"));
            ui->actionRun->setChecked(false);
            run();
            return;
        }

        gSimulation->UpdateSimulation();
        stepCount++;

        if ((stepCount % skip) == 0)
        {
            if (trackingFlag)
            {
                Body *body = gSimulation->GetBody(gSimulation->GetInterface()->TrackBodyID.c_str());
                if (body)
                {
                    const double *position = dBodyGetPosition(body->GetBodyID());
                    ui->widgetGLWidget->SetCameraCOIX(position[0] + trackingOffset);
                    allowGLUpdateFlag = false;
                    ui->doubleSpinBoxCOIX->setValue(position[0] + trackingOffset);
                    allowGLUpdateFlag = true;
                }
            }
            if (stepFlag)
            {
                stepFlag = false;
                timer->stop();
            }
            updateGL();
            if (movieFlag)
            {
                QString movieFullPath = QString("%1/%2").arg(configFile.absolutePath()).arg(movieFolder);
                if (QFile::exists(movieFullPath) == false)
                {
                    QDir dir("/");
                    dir.mkpath(movieFullPath);
                }
                QString filename = QString("%1/%2%3").arg(movieFullPath).arg("Frame").arg(gSimulation->GetTime(), 12, 'f', 7, QChar('0'));
                ui->widgetGLWidget->WriteFrame(filename, ui->widgetGLWidget->GetMovieFormat());
            }
            QString time = QString("%1").arg(gSimulation->GetTime(), 0, 'f', 5);
            ui->lcdNumberTime->display(time);
        }

        if (gSimulation->ShouldQuit())
        {
            statusBar()->showMessage(tr("Simulation ended normally"));
            ui->textEditLog->append(QString("Fitness = %1\n").arg(gSimulation->CalculateInstantaneousFitness(), 0, 'f', 5));
            updateGL();
            QString time = QString("%1").arg(gSimulation->GetTime(), 0, 'f', 5);
            ui->lcdNumberTime->display(time);
            ui->actionRun->setChecked(false);
            run();
            return;
        }
        if (gSimulation->TestForCatastrophy())
        {
            statusBar()->showMessage(tr("Simulation aborted"));
            ui->textEditLog->append(QString("Fitness = %1\n").arg(gSimulation->CalculateInstantaneousFitness(), 0, 'f', 5));
            updateGL();
            QString time = QString("%1").arg(gSimulation->GetTime(), 0, 'f', 5);
            ui->lcdNumberTime->display(time);
            ui->actionRun->setChecked(false);
            run();
            return;
        }
    }
}

void MainWindow::snapshot()
{
    int count = 0;
    QDir dir(configFile.absolutePath());
    QStringList list = dir.entryList(QDir::Files, QDir::Name);
    QStringList matches = list.filter(QRegExp(QString("^Snapshot\\d\\d\\d\\d\\d\\..*")));
    if (matches.size() > 0)
    {
        QString numberString = matches.last().mid(8, 5);
        count = numberString.toInt() + 1;
    }
    QString filename = QString("Snapshot%1").arg(count, 5, 10, QChar('0'));
    GLWidget::MovieFormat format = ui->widgetGLWidget->GetMovieFormat();
    if (format == GLWidget::Quicktime) format = GLWidget::TIFF; // can't write a snapshot in Quicktime format so fall back on TIFF
    ui->widgetGLWidget->WriteFrame(QString("%1/%2").arg(configFile.absolutePath()).arg(filename), format);
    statusBar()->showMessage(tr("Snapshot taken"));
}

void MainWindow::fillVisibitilityLists()
{
    if (gSimulation == 0) return;

    QListWidgetItem *item;
    int count;
    std::map<std::string, Body *> *bodyList = gSimulation->GetBodyList();
    std::map<std::string, Joint *> *jointList = gSimulation->GetJointList();
    std::map<std::string, Geom *> *geomList = gSimulation->GetGeomList();
    std::map<std::string, Muscle *> *muscleList = gSimulation->GetMuscleList();

    count = 0;
    ui->listWidgetBody->clear();
    std::map<std::string, Body *>::const_iterator bodyIterator;
    for (bodyIterator = bodyList->begin(); bodyIterator != bodyList->end(); bodyIterator++)
    {
        ui->listWidgetBody->addItem(bodyIterator->first.c_str());
        item = ui->listWidgetBody->item(count++);
        item->setCheckState(Qt::Checked);
    }

    count = 0;
    ui->listWidgetJoint->clear();
    std::map<std::string, Joint *>::const_iterator jointIterator;
    for (jointIterator = jointList->begin(); jointIterator != jointList->end(); jointIterator++)
    {
        ui->listWidgetJoint->addItem(jointIterator->first.c_str());
        item = ui->listWidgetJoint->item(count++);
        item->setCheckState(Qt::Checked);
    }

    count = 0;
    ui->listWidgetGeom->clear();
    std::map<std::string, Geom *>::const_iterator geomIterator;
    for (geomIterator = geomList->begin(); geomIterator != geomList->end(); geomIterator++)
    {
        ui->listWidgetGeom->addItem(geomIterator->first.c_str());
        item = ui->listWidgetGeom->item(count++);
        item->setCheckState(Qt::Checked);
    }

    count = 0;
    ui->listWidgetMuscle->clear();
    std::map<std::string, Muscle *>::const_iterator muscleIterator;
    for (muscleIterator = muscleList->begin(); muscleIterator != muscleList->end(); muscleIterator++)
    {
        ui->listWidgetMuscle->addItem(muscleIterator->first.c_str());
        item = ui->listWidgetMuscle->item(count++);
        item->setCheckState(Qt::Checked);
    }


}

void MainWindow::buttonCameraRight()
{
    ui->widgetGLWidget->SetCameraRight();
    updateGL();
}


void MainWindow::buttonCameraTop()
{
    ui->widgetGLWidget->SetCameraTop();
    updateGL();
}


void MainWindow::buttonCameraFront()
{
    ui->widgetGLWidget->SetCameraFront();
    updateGL();
}


void MainWindow::spinboxDistanceChanged(double v)
{
    ui->widgetGLWidget->SetCameraDistance(v);
    updateGL();
}


void MainWindow::spinboxFoVChanged(double v)
{
    ui->widgetGLWidget->SetCameraFoV(v);
    updateGL();
}


void MainWindow::spinboxCOIXChanged(double v)
{
    ui->widgetGLWidget->SetCameraCOIX(v);
    updateGL();
}


void MainWindow::spinboxCOIYChanged(double v)
{
    ui->widgetGLWidget->SetCameraCOIY(v);
    updateGL();
}


void MainWindow::spinboxCOIZChanged(double v)
{
    ui->widgetGLWidget->SetCameraCOIZ(v);
    updateGL();
}


void MainWindow::checkboxTracking(int v)
{
    trackingFlag = v;
    if (gSimulation)
    {
        if (trackingFlag)
        {
            Body *body = gSimulation->GetBody(gSimulation->GetInterface()->TrackBodyID.c_str());
            if (body)
            {
                const double *position = dBodyGetPosition(body->GetBodyID());
                ui->widgetGLWidget->SetCameraCOIX(position[0] + trackingOffset);
                ui->doubleSpinBoxCOIX->setValue(position[0] + trackingOffset);
                updateGL();
            }
        }
    }
}

void MainWindow::lineeditTrackingOffset()
{
    bool ok;
    trackingOffset = ui->lineEditTrackingOffset->text().toDouble(&ok);
    if (ok == false)
        ui->lineEditTrackingOffset->setText(QString("%1").arg(trackingOffset));
    if (gSimulation)
    {
        if (trackingFlag)
        {
            Body *body = gSimulation->GetBody(gSimulation->GetInterface()->TrackBodyID.c_str());
            if (body)
            {
                const double *position = dBodyGetPosition(body->GetBodyID());
                ui->widgetGLWidget->SetCameraCOIX(position[0] + trackingOffset);
                ui->doubleSpinBoxCOIX->setValue(position[0] + trackingOffset);
                updateGL();
            }
        }
    }
}


void MainWindow::checkboxOverlay(int v)
{
    ui->widgetGLWidget->SetOverlay(v);
    updateGL();
}


void MainWindow::checkboxContactForce(int v)
{
    gDrawContactForces = v;
    updateGL();
}


void MainWindow::checkboxMuscleForce(int v)
{
    gDrawMuscleForces = v;
    updateGL();
}


void MainWindow::checkboxFramerate(int v)
{
    if (v) ui->widgetGLWidget->SetDisplayFramerate(true);
    else ui->widgetGLWidget->SetDisplayFramerate(false);
    updateGL();
}


void MainWindow::checkboxActivationColours(int v)
{
    gActivationDisplay = v;
    if (gSimulation)
    {
        std::map<std::string, Muscle *> *muscleList = gSimulation->GetMuscleList();
        std::map<std::string, Muscle *>::const_iterator muscleIter;
        for (muscleIter = muscleList->begin(); muscleIter != muscleList->end(); muscleIter++)
        {
            muscleIter->second->GetStrap()->SetColour(gSimulation->GetInterface()->StrapColour);
            muscleIter->second->GetStrap()->SetForceColour(gSimulation->GetInterface()->StrapForceColour);
            muscleIter->second->GetStrap()->SetLastDrawTime(-1);
        }
    }
    updateGL();
}

void MainWindow::checkboxOpenCL(int v)
{
    g_UseOpenCL = v;
}


void MainWindow::lineeditMovieFolder(QString folder)
{
    movieFolder = folder;
}


void MainWindow::radioPPM(bool v)
{
    if (v)
    {
        ui->widgetGLWidget->SetMovieFormat(GLWidget::PPM);
    }
}


void MainWindow::radioTIFF(bool v)
{
   if (v)
    {
        ui->widgetGLWidget->SetMovieFormat(GLWidget::TIFF);
    }
}


void MainWindow::radioPOVRay(bool v)
{
   if (v)
    {
        ui->widgetGLWidget->SetMovieFormat(GLWidget::POVRay);
    }
}


void MainWindow::radioOBJ(bool v)
{
   if (v)
    {
        ui->widgetGLWidget->SetMovieFormat(GLWidget::OBJ);
    }
}

void MainWindow::radioQuicktime(bool v)
{
   if (v)
    {
        ui->widgetGLWidget->SetMovieFormat(GLWidget::Quicktime);
    }
}


void MainWindow::spinboxSkip(int v)
{
    skip = v;
}


void MainWindow::spinboxTimeMax(double v)
{
    gSimulation->SetTimeLimit(v);
}


void MainWindow::listMuscleChecked(QListWidgetItem* item)
{
    bool visible = true;
    if (item->checkState() == Qt::Unchecked) visible = false;
    (*gSimulation->GetMuscleList())[std::string(item->text().toUtf8())]->SetAllVisible(visible);
    updateGL();
}


void MainWindow::listBodyChecked(QListWidgetItem* item)
{
    bool visible = true;
    if (item->checkState() == Qt::Unchecked) visible = false;
    (*gSimulation->GetBodyList())[std::string(item->text().toUtf8())]->SetVisible(visible);
    updateGL();
}


void MainWindow::listJointChecked(QListWidgetItem* item)
{
    bool visible = true;
    if (item->checkState() == Qt::Unchecked) visible = false;
    (*gSimulation->GetJointList())[std::string(item->text().toUtf8())]->SetVisible(visible);
    updateGL();
}


void MainWindow::listGeomChecked(QListWidgetItem* item)
{
   bool visible = true;
    if (item->checkState() == Qt::Unchecked) visible = false;
    (*gSimulation->GetGeomList())[std::string(item->text().toUtf8())]->SetVisible(visible);
    updateGL();
}

void MainWindow::menuPreferences()
{
    DialogPreferences dialogPreferences(this);
    dialogPreferences.SetValues(*preferences);


    //glWidget->releaseKeyboard();
    int status = dialogPreferences.exec();
    //glWidget->grabKeyboard();

    if (status == QDialog::Accepted) // write the new settings
    {
        dialogPreferences.GetValues(preferences);
        writeSettings();

        // these settings have immediate effect
        ui->widgetGLWidget->SetYUp(preferences->YUp);
        //setDefaultLights(preferences->YUp);
        ui->widgetGLWidget->Set3DCursorRadius(preferences->CursorRadius);
        ui->widgetGLWidget->Set3DCursorNudge(preferences->Nudge);
        ui->widgetGLWidget->SetCameraFrontClip(preferences->CameraFrontClip);
        ui->widgetGLWidget->SetCameraBackClip(preferences->CameraBackClip);

        updateGL();

    }

}

void MainWindow::menuOutputs()
{
    if (gSimulation == 0) return;

    DialogOutputSelect dialogOutputSelect(this);

    //glWidget->releaseKeyboard();
    int status = dialogOutputSelect.exec();
    //glWidget->grabKeyboard();

    QListWidgetItem *item;
    int i;
    bool dump;

    if (status == QDialog::Accepted) // write the new settings
    {
        for (i = 0; dialogOutputSelect.listWidgetBody && i < dialogOutputSelect.listWidgetBody->count(); i++)
        {
            item = dialogOutputSelect.listWidgetBody->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetBodyList())[std::string(item->text().toUtf8())]->SetDump(dump);
        }

        for (i = 0; dialogOutputSelect.listWidgetMuscle && i < dialogOutputSelect.listWidgetMuscle->count(); i++)
        {
            item = dialogOutputSelect.listWidgetMuscle->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetMuscleList())[std::string(item->text().toUtf8())]->SetDump(dump);
            (*gSimulation->GetMuscleList())[std::string(item->text().toUtf8())]->GetStrap()->SetDump(dump);
        }

        for (i = 0; dialogOutputSelect.listWidgetGeom && i < dialogOutputSelect.listWidgetGeom->count(); i++)
        {
            item = dialogOutputSelect.listWidgetGeom->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetGeomList())[std::string(item->text().toUtf8())]->SetDump(dump);
        }

        for (i = 0; dialogOutputSelect.listWidgetJoint && i < dialogOutputSelect.listWidgetJoint->count(); i++)
        {
            item = dialogOutputSelect.listWidgetJoint->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetJointList())[std::string(item->text().toUtf8())]->SetDump(dump);
        }

        for (i = 0; dialogOutputSelect.listWidgetDriver && i < dialogOutputSelect.listWidgetDriver->count(); i++)
        {
            item = dialogOutputSelect.listWidgetDriver->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetDriverList())[std::string(item->text().toUtf8())]->SetDump(dump);
        }

        for (i = 0; dialogOutputSelect.listWidgetDataTarget && i < dialogOutputSelect.listWidgetDataTarget->count(); i++)
        {
            item = dialogOutputSelect.listWidgetDataTarget->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetDataTargetList())[std::string(item->text().toUtf8())]->SetDump(dump);
        }

        for (i = 0; dialogOutputSelect.listWidgetReporter && i < dialogOutputSelect.listWidgetReporter->count(); i++)
        {
            item = dialogOutputSelect.listWidgetReporter->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetReporterList())[std::string(item->text().toUtf8())]->SetDump(dump);
        }

        for (i = 0; dialogOutputSelect.listWidgetWarehouse && i < dialogOutputSelect.listWidgetWarehouse->count(); i++)
        {
            item = dialogOutputSelect.listWidgetWarehouse->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetWarehouseList())[std::string(item->text().toUtf8())]->SetDump(dump);
        }

    }
}

void MainWindow::menuInterface()
{
    DialogInterface dialogInterface(this);

    // hook up the preferences dialog to the GLWidget
    connect(&dialogInterface, SIGNAL(EmitBackgroundColour(float, float, float, float)), ui->widgetGLWidget, SLOT(SetBackground(float, float, float, float)));
    connect(&dialogInterface, SIGNAL(EmitRedrawRequired()), ui->widgetGLWidget, SLOT(update()));

    dialogInterface.SetValues(*preferences);

    //glWidget->releaseKeyboard();
    int status = dialogInterface.exec();
    //glWidget->grabKeyboard();

    if (status == QDialog::Accepted) // write the new settings
    {
        dialogInterface.GetValues(preferences);
        writeSettings();

        // currently these values only take effect at startup so no need to update
        // updateGL();
    }

}


void MainWindow::setInterfaceValues()
{
    ui->widgetGLWidget->SetCameraDistance(preferences->CameraDistance);
    ui->widgetGLWidget->SetCameraFoV(preferences->CameraFoV);
    ui->widgetGLWidget->SetCameraCOIX(preferences->CameraCOIX);
    ui->widgetGLWidget->SetCameraCOIY(preferences->CameraCOIY);
    ui->widgetGLWidget->SetCameraCOIZ(preferences->CameraCOIZ);
    ui->widgetGLWidget->SetCameraFrontClip(preferences->CameraFrontClip);
    ui->widgetGLWidget->SetCameraBackClip(preferences->CameraBackClip);

    trackingFlag = preferences->DisplayTracking;
    ui->widgetGLWidget->SetOverlay(preferences->DisplayOverlay);
    gDrawContactForces = preferences->DisplayContactForces;
    gDrawMuscleForces = preferences->DisplayMuscleForces;
    ui->widgetGLWidget->SetDisplayFramerate(preferences->DisplayFramerate);
    ui->widgetGLWidget->SetBackground(preferences->BackgroundColour.redF(), preferences->BackgroundColour.greenF(), preferences->BackgroundColour.blueF(), preferences->BackgroundColour.alphaF());
    gActivationDisplay = preferences->DisplayActivation;

    movieFolder = preferences->MoviePath;
    skip = preferences->MovieSkip;
    ui->widgetGLWidget->SetMovieFormat(static_cast<GLWidget::MovieFormat>(preferences->MovieFormat));

    ui->widgetGLWidget->SetYUp(preferences->YUp);

    ui->widgetGLWidget->Set3DCursorRadius(preferences->CursorRadius);
    ui->widgetGLWidget->Set3DCursorNudge(preferences->Nudge);

#ifdef USE_OPENCL
    g_UseOpenCL = preferences->OpenCLUseOpenCL;
#else
    g_UseOpenCL = 0;
#endif

    ui->doubleSpinBoxDistance->setValue(preferences->CameraDistance);
    ui->doubleSpinBoxFoV->setValue(preferences->CameraFoV);
    ui->doubleSpinBoxCOIX->setValue(preferences->CameraCOIX);
    ui->doubleSpinBoxCOIY->setValue(preferences->CameraCOIY);
    ui->doubleSpinBoxCOIZ->setValue(preferences->CameraCOIZ);

    ui->checkBoxTracking->setChecked(preferences->DisplayTracking);
    ui->checkBoxOverlay->setChecked(preferences->DisplayOverlay);
    ui->checkBoxContactForce->setChecked(preferences->DisplayContactForces);
    ui->checkBoxMuscleForce->setChecked(preferences->DisplayMuscleForces);
    ui->checkBoxFramerate->setChecked(preferences->DisplayFramerate);
    ui->checkBoxActivationColours->setChecked(preferences->DisplayActivation);

    ui->lineEditMovieFolder->setText(preferences->MoviePath);
    ui->spinBoxSkip->setValue(preferences->MovieSkip);
    ui->radioButtonPPM->setChecked(static_cast<GLWidget::MovieFormat>(preferences->MovieFormat) == GLWidget::PPM);
    ui->radioButtonTIFF->setChecked(static_cast<GLWidget::MovieFormat>(preferences->MovieFormat) == GLWidget::TIFF);
    ui->radioButtonPOVRay->setChecked(static_cast<GLWidget::MovieFormat>(preferences->MovieFormat) == GLWidget::POVRay);
    ui->radioButtonOBJ->setChecked(static_cast<GLWidget::MovieFormat>(preferences->MovieFormat) == GLWidget::OBJ);

    /*
    ui->widgetGLWidget->SetCameraDistance(settings->value("CameraDistance", 50).toDouble());
    ui->widgetGLWidget->SetCameraFoV(settings->value("CameraFoV", 5).toDouble());
    ui->widgetGLWidget->SetCameraCOIX(settings->value("CameraCOIX", 0).toDouble());
    ui->widgetGLWidget->SetCameraCOIY(settings->value("CameraCOIY", 0).toDouble());
    ui->widgetGLWidget->SetCameraCOIZ(settings->value("CameraCOIZ", 0).toDouble());
    ui->widgetGLWidget->SetCameraFrontClip(settings->value("CameraFrontClip", 1).toDouble());
    ui->widgetGLWidget->SetCameraBackClip(settings->value("CameraBackClip", 100).toDouble());

    trackingFlag = settings->value("DisplayTracking", 0).toBool();
    ui->widgetGLWidget->SetOverlay(settings->value("DisplayOverlay", 0).toBool());
    gDrawContactForces = settings->value("DisplayContactForces", 0).toBool();
    gDrawMuscleForces = settings->value("DisplayMuscleForces", 0).toBool();
    ui->widgetGLWidget->SetDisplayFramerate(settings->value("DisplayFramerate", 0).toBool());
    g_ActivationDisplay = settings->value("DisplayActivation", 0).toBool();

    movieFolder = settings->value("MoviePath", QString("movie")).toString();
    skip = settings->value("MovieSkip", 1).toInt();
    ui->widgetGLWidget->SetMovieFormat(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()));

    ui->widgetGLWidget->SetYUp(settings->value("YUp", 0).toBool());

    ui->widgetGLWidget->Set3DCursorRadius(settings->value("CursorRadius", 0).toDouble());
    ui->widgetGLWidget->Set3DCursorNudge(settings->value("Nudge", 0).toDouble());

#ifdef USE_OPENCL
    g_UseOpenCL = settings->value("OpenCLUseOpenCL", 0).toInt();
#else
    g_UseOpenCL = 0;
#endif

    ui->doubleSpinBoxDistance->setValue(settings->value("CameraDistance", 50).toDouble());
    ui->doubleSpinBoxFoV->setValue(settings->value("CameraFoV", 5).toDouble());
    ui->doubleSpinBoxCOIX->setValue(settings->value("CameraCOIX", 0).toDouble());
    ui->doubleSpinBoxCOIY->setValue(settings->value("CameraCOIY", 0).toDouble());
    ui->doubleSpinBoxCOIZ->setValue(settings->value("CameraCOIZ", 0).toDouble());

    ui->checkBoxTracking->setChecked(settings->value("DisplayTracking", 0).toBool());
    ui->checkBoxOverlay->setChecked(settings->value("DisplayOverlay", 0).toBool());
    ui->checkBoxContactForce->setChecked(settings->value("DisplayContactForces", 0).toBool());
    ui->checkBoxMuscleForce->setChecked(settings->value("DisplayMuscleForces", 0).toBool());
    ui->checkBoxWhiteBackground->setChecked(settings->value("DisplayWhiteBackground", 0).toBool());
    ui->checkBoxFramerate->setChecked(settings->value("DisplayFramerate", 0).toBool());
    ui->checkBoxActivationColours->setChecked(settings->value("DisplayActivation", 0).toBool());
    ui->checkBoxOpenCL->setChecked(settings->value("OpenCLUseOpenCL", 0).toBool());

    ui->lineEditMovieFolder->setText(settings->value("MoviePath", QString("movie")).toString());
    ui->spinBoxSkip->setValue(settings->value("MovieSkip", 1).toInt());
    ui->radioButtonPPM->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::PPM);
    ui->radioButtonTIFF->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::TIFF);
    ui->radioButtonPOVRay->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::POVRay);
    ui->radioButtonOBJ->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::OBJ);
*/
}

void MainWindow::writeSettings()
{
    preferences->CameraDistance = ui->widgetGLWidget->GetCameraDistance();
    preferences->CameraFoV = ui->widgetGLWidget->GetCameraFoV();
    preferences->CameraCOIX = ui->widgetGLWidget->GetCameraCOIX();
    preferences->CameraCOIY = ui->widgetGLWidget->GetCameraCOIY();
    preferences->CameraCOIZ = ui->widgetGLWidget->GetCameraCOIZ();
    GLfloat x, y, z;
    ui->widgetGLWidget->GetCameraVec(&x, &y, &z);
    preferences->CameraVecX = x;
    preferences->CameraVecY = y;
    preferences->CameraVecZ = z;
    ui->widgetGLWidget->GetCameraUp(&x, &y, &z);
    preferences->CameraUpX = x;
    preferences->CameraUpY = y;
    preferences->CameraUpZ = z;

    preferences->MainWindowGeometry = saveGeometry();
    preferences->MainWindowState = saveState();
    preferences->SplitterState = ui->splitter->saveState();
    preferences->Write();
}


void MainWindow::menuRequestMuscle(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = ui->listWidgetMuscle->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool visible;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            visible = true;
        }
        else
        {
            state = Qt::Unchecked;
            visible = false;
        }
        for (i = 0; i < ui->listWidgetMuscle->count(); i++)
        {
            item = ui->listWidgetMuscle->item(i);
            item->setCheckState(state);
            (*gSimulation->GetMuscleList())[std::string(item->text().toUtf8())]->SetAllVisible(visible);
        }
        updateGL();
    }
}

void MainWindow::menuRequestBody(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = ui->listWidgetBody->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool visible;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            visible = true;
        }
        else
        {
            state = Qt::Unchecked;
            visible = false;
        }
        for (i = 0; i < ui->listWidgetBody->count(); i++)
        {
            item = ui->listWidgetBody->item(i);
            item->setCheckState(state);
            (*gSimulation->GetBodyList())[std::string(item->text().toUtf8())]->SetVisible(visible);
        }
        updateGL();
    }
}

void MainWindow::menuRequestJoint(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = ui->listWidgetJoint->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool visible;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            visible = true;
        }
        else
        {
            state = Qt::Unchecked;
            visible = false;
        }
        for (i = 0; i < ui->listWidgetJoint->count(); i++)
        {
            item = ui->listWidgetJoint->item(i);
            item->setCheckState(state);
            (*gSimulation->GetJointList())[std::string(item->text().toUtf8())]->SetVisible(visible);
        }
        updateGL();
    }
}

void MainWindow::menuRequestGeom(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = ui->listWidgetGeom->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool visible;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            visible = true;
        }
        else
        {
            state = Qt::Unchecked;
            visible = false;
        }
        for (i = 0; i < ui->listWidgetGeom->count(); i++)
        {
            item = ui->listWidgetGeom->item(i);
            item->setCheckState(state);
            (*gSimulation->GetGeomList())[std::string(item->text().toUtf8())]->SetVisible(visible);
        }
        updateGL();
    }
}

void MainWindow::menuDefaultView()
{
    trackingOffset = preferences->DefaultTrackingOffset;
    ui->lineEditTrackingOffset->setText(QString("%1").arg(trackingOffset));

    ui->widgetGLWidget->SetCameraDistance(preferences->DefaultCameraDistance);
    ui->widgetGLWidget->SetCameraFoV(preferences->DefaultCameraFoV);
    ui->widgetGLWidget->SetCameraCOIX(preferences->DefaultCameraCOIX);
    ui->widgetGLWidget->SetCameraCOIY(preferences->DefaultCameraCOIY);
    ui->widgetGLWidget->SetCameraCOIZ(preferences->DefaultCameraCOIZ);

    ui->widgetGLWidget->SetCameraVec(preferences->DefaultCameraVecX, preferences->DefaultCameraVecY, preferences->DefaultCameraVecZ);
    ui->widgetGLWidget->SetCameraUp(preferences->DefaultCameraUpX, preferences->DefaultCameraUpY, preferences->DefaultCameraUpZ);

    ui->doubleSpinBoxDistance->setValue(preferences->DefaultCameraDistance);
    ui->doubleSpinBoxFoV->setValue(preferences->DefaultCameraFoV);
    ui->doubleSpinBoxCOIX->setValue(preferences->DefaultCameraCOIX);
    ui->doubleSpinBoxCOIY->setValue(preferences->DefaultCameraCOIY);
    ui->doubleSpinBoxCOIZ->setValue(preferences->DefaultCameraCOIZ);

    updateGL();
}

void MainWindow::menuSaveDefaultView()
{
    preferences->DefaultCameraDistance = ui->doubleSpinBoxDistance->value();
    preferences->DefaultCameraFoV = ui->doubleSpinBoxFoV->value();
    preferences->DefaultCameraCOIX = ui->doubleSpinBoxCOIX->value();
    preferences->DefaultCameraCOIY = ui->doubleSpinBoxCOIY->value();
    preferences->DefaultCameraCOIZ = ui->doubleSpinBoxCOIZ->value();
    preferences->DefaultTrackingOffset = ui->lineEditTrackingOffset->text().toDouble();

    GLfloat x, y, z;
    ui->widgetGLWidget->GetCameraVec(&x, &y, &z);
    preferences->DefaultCameraVecX = double(x);
    preferences->DefaultCameraVecY = double(y);
    preferences->DefaultCameraVecZ = double(z);
    ui->widgetGLWidget->GetCameraUp(&x, &y, &z);
    preferences->DefaultCameraUpX = double(x);
    preferences->DefaultCameraUpY = double(y);
    preferences->DefaultCameraUpZ = double(z);
}

void MainWindow::setStatusString(QString s)
{
    statusBar()->showMessage(s);
}

void MainWindow::setUICOI(double x, double y, double z)
{
    ui->doubleSpinBoxCOIX->setValue(x);
    ui->doubleSpinBoxCOIY->setValue(y);
    ui->doubleSpinBoxCOIZ->setValue(z);
}

void MainWindow::setUIFoV(double v)
{
    ui->doubleSpinBoxFoV->setValue(v);
}

/*
void MainWindow::setDefaultLights(bool yUp)
{
    // lights
    Light l0, l1, l2, l3;

    // Green front right
    l0.SetAmbient (0.2, 0.2, 0.2, 1);
    l0.SetDiffuse (0.3, 0.6, 0.3, 1);
    l0.SetSpecular(0.4, 0.6, 0.4, 1);
    if (yUp) l0.SetPosition(0.5, 1, 1, 0);
    else l0.SetPosition(0.5, -1, 1, 0);

    // Blue front left
    l1.SetAmbient (0.2, 0.2, 0.2, 1);
    l1.SetDiffuse (0.3, 0.3, 0.6, 1);
    l1.SetSpecular(0.4, 0.4, 0.6, 1);
    if (yUp) l1.SetPosition(0.5, 1, -1, 0);
    else l1.SetPosition(0.5, 1, 1, 0);

    // Red back centre
    l2.SetAmbient (0.2, 0.2, 0.2, 1);
    l2.SetDiffuse (0.6, 0.3, 0.3, 1);
    l2.SetSpecular(0.6, 0.4, 0.4, 1);
    if (yUp) l2.SetPosition(-1, 0, 0, 0);
    else l2.SetPosition(-1, 0, 0, 0);

    // White underneath
    l3.SetAmbient (0.1, 0.1, 0.1, 1);
    l3.SetDiffuse (0.2, 0.2, 0.2, 1);
    l3.SetSpecular(0.3, 0.3, 0.3, 1);
    if (yUp) l3.SetPosition(0, -1, 0, 0);
    else l3.SetPosition(0, 0, -1, 0);

    ui->widgetGLWidget->ClearLights();
    ui->widgetGLWidget->AddLight(&l0);
    ui->widgetGLWidget->AddLight(&l1);
    ui->widgetGLWidget->AddLight(&l2);
    ui->widgetGLWidget->AddLight(&l3);
}
*/

void MainWindow::menu640x480()
{
    int w = 640, h = 480;
    int deltaW = w - ui->widgetGLWidget->width();
    int deltaH = h - ui->widgetGLWidget->height();
    resize(width() + deltaW, height() + deltaH);
    std::cerr << "ui->widgetGLWidget->width()=" << ui->widgetGLWidget->width() << " ui->widgetGLWidget->height()=" << ui->widgetGLWidget->height() << "\n";
}

void MainWindow::menu800x600()
{
    int w = 800, h = 600;
    int deltaW = w - ui->widgetGLWidget->width();
    int deltaH = h - ui->widgetGLWidget->height();
    resize(width() + deltaW, height() + deltaH);
}

void MainWindow::menu1280x720()
{
    int w = 1280, h = 720;
    int deltaW = w - ui->widgetGLWidget->width();
    int deltaH = h - ui->widgetGLWidget->height();
    resize(width() + deltaW, height() + deltaH);
}

void MainWindow::menu1920x1080()
{
    int w = 1920, h = 1080;
    int deltaW = w - ui->widgetGLWidget->width();
    int deltaH = h - ui->widgetGLWidget->height();
    resize(width() + deltaW, height() + deltaH);
}

void MainWindow::resizeAndCentre(int w, int h)
{
    QDesktopWidget *desktop = qApp->desktop();
    QRect available = desktop->availableGeometry(-1);

    // Need to find how big the central widget is compared to the window
    int heightDiff = height() - ui->widgetGLWidget->height();
    int widthDiff = width() - ui->widgetGLWidget->width();
    int newWidth = w + widthDiff;
    int newHeight = h + heightDiff;

    // centre window
    int topLeftX = available.left() + (available.width() / 2) - (newWidth / 2);
    int topLeftY = available.top() + (available.height() / 2) - (newHeight / 2);
    // but don't start off screen
    if (topLeftX < available.left()) topLeftX = available.left();
    if (topLeftY < available.top()) topLeftY = available.top();

    move(topLeftX, topLeftY);
    resize(newWidth, newHeight);
    updateGL();
}

void MainWindow::menuStartQuicktimeSave()
{
    QFileInfo info = preferences->LastFileOpened;

    QString fileName = CustomFileDialogs::getSaveFileName(this, tr("Save output as Quicktime file"), info.absolutePath(), tr("Quicktime Files (*.mov)"), 0, QFileDialog::DontUseNativeDialog);

    if (fileName.isNull() == false)
    {
        MyMovieHandle qtMovie = initialiseMovieFile(fileName.toUtf8(), preferences->QuicktimeFramerate);
        if (qtMovie)
        {
            ui->widgetGLWidget->SetQTMovie(qtMovie);
        }
    }
}

void MainWindow::menuStopQuicktimeSave()
{
    MyMovieHandle qtMovie = ui->widgetGLWidget->GetQTMovie();
    if (qtMovie) closeMovieFile(qtMovie);
    qtMovie = 0;
    ui->widgetGLWidget->SetQTMovie(qtMovie);
}

void MainWindow::log(const char *text)
{
    ui->textEditLog->append(text);
}

void MainWindow::copy()
{
    QWidget* focused = QApplication::focusWidget();
    if( focused != 0 )
    {
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyPress, Qt::Key_C, Qt::ControlModifier ));
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyRelease, Qt::Key_C, Qt::ControlModifier ));
    }

}

void MainWindow::cut()
{
    QWidget* focused = QApplication::focusWidget();
    if( focused != 0 )
    {
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyPress, Qt::Key_X, Qt::ControlModifier ));
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyRelease, Qt::Key_X, Qt::ControlModifier ));
    }

}

void MainWindow::paste()
{
    QWidget* focused = QApplication::focusWidget();
    if( focused != 0 )
    {
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyPress, Qt::Key_V, Qt::ControlModifier ));
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyRelease, Qt::Key_V, Qt::ControlModifier ));
    }

}

void MainWindow::selectAll()
{
    QWidget* focused = QApplication::focusWidget();
    if( focused != 0 )
    {
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyPress, Qt::Key_A, Qt::ControlModifier ));
        QApplication::postEvent( focused, new QKeyEvent( QEvent::KeyRelease, Qt::Key_A, Qt::ControlModifier ));
    }

}

void MainWindow::updateGL()
{
    if (allowGLUpdateFlag) ui->widgetGLWidget->update();
}


//void MainWindow::glwidgetGrabKeyboard()
//{
//    glWidget->grabKeyboard();
//}

//void MainWindow::glwidgetReleaseKeyboard()
//{
//    glWidget->releaseKeyboard();
//}

QByteArray MainWindow::readResource(const QString &resource)
{
    QFile file(resource);
    bool ok = file.open(QIODevice::ReadOnly);
    Q_ASSERT(ok);
    return file.readAll();
}

void MainWindow::menuStartWarehouseExport()
{
    if (gSimulation == 0) return;

    QFileInfo info = preferences->LastFileOpened;
    QString fileName = CustomFileDialogs::getSaveFileName(this, tr("Save output as Warehouse file"), info.absolutePath(), tr("Text Files (*.txt)"), 0, QFileDialog::DontUseNativeDialog);

    if (fileName.isNull() == false)
    {
        ui->actionStart_Warehouse_Export->setEnabled(false);
        ui->actionStop_Warehouse_Export->setEnabled(true);
        gSimulation->SetOutputWarehouseFile(fileName.toUtf8());
    }
}

void MainWindow::menuStopWarehouseExport()
{
    if (gSimulation == 0) return;

    ui->actionStart_Warehouse_Export->setEnabled(true);
    ui->actionStop_Warehouse_Export->setEnabled(false);
    gSimulation->SetOutputWarehouseFile(0);
}

void MainWindow::menuImportWarehouse()
{
    if (gSimulation == 0) return;
    QString fileName = CustomFileDialogs::getOpenFileName(this, tr("Open Warehouse File"), "", tr("Warehouse Files (*.txt)"), 0, QFileDialog::DontUseNativeDialog);

    if (fileName.isNull() == false)
    {
        gSimulation->AddWarehouse(fileName.toUtf8());
        statusBar()->showMessage(QString("Warehouse %1 added").arg(fileName));
    }
}

void MainWindow::menuToggleFullScreen()
{
    setWindowState(windowState() ^ Qt::WindowFullScreen);
}
