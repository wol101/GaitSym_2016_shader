/*
 *  Preferences.cpp
 *  GaitSym
 *
 *  Created by Bill Sellers on 12/06/2014.
 *  Copyright 2014 Bill Sellers. All rights reserved.
 *
 */

#include "preferences.h"

#include <QSettings>

Preferences::Preferences()
{
    Organisation = "AnimalSimulationLaboratory";
    Application = "GaitSym2016";
}

void Preferences::Read()
{
    QSettings settings(Organisation, Application);

    MainWindowGeometry = settings.value("MainWindowGeometry").toByteArray();
    MainWindowState = settings.value("MainWindowState").toByteArray();
    SplitterState = settings.value("SplitterState").toByteArray();

    CameraDistance = settings.value("CameraDistance", 50).toDouble();
    CameraFoV = settings.value("CameraFoV", 5).toDouble();
    CameraCOIX = settings.value("CameraCOIX", 0).toDouble();
    CameraCOIY = settings.value("CameraCOIY", 0).toDouble();
    CameraCOIZ = settings.value("CameraCOIZ", 0).toDouble();
    CameraVecX = settings.value("CameraVecX", 0).toDouble();
    CameraVecY = settings.value("CameraVecY", 1).toDouble();
    CameraVecZ = settings.value("CameraVecZ", 0).toDouble();
    CameraUpX = settings.value("CameraUpX", 0).toDouble();
    CameraUpY = settings.value("CameraUpY", 0).toDouble();
    CameraUpZ = settings.value("CameraUpZ", 1).toDouble();
    TrackingOffset = settings.value("TrackingOffset", 0).toDouble();
    TrackingFlag = settings.value("TrackingFlag", false).toBool();
    YUp = settings.value("YUp", false).toBool();
    YUp = settings.value("OrthographicFlag", true).toBool();

    FrameSkip = settings.value("FrameSkip", 1000).toInt();
    EnvironmentX = settings.value("EnvironmentX", 1).toDouble();
    EnvironmentY = settings.value("EnvironmentY", 1).toDouble();
    EnvironmentZ = settings.value("EnvironmentZ", 1).toDouble();
    BodyX = settings.value("BodyX", 0.1).toDouble();
    BodyY = settings.value("BodyY", 0.1).toDouble();
    BodyZ = settings.value("BodyZ", 0.1).toDouble();
    JointX = settings.value("JointX", 0.05).toDouble();
    JointY = settings.value("JointY", 0.05).toDouble();
    JointZ = settings.value("JointZ", 0.05).toDouble();
    GeomX = settings.value("GeomX", 0.05).toDouble();
    GeomY = settings.value("GeomY", 0.05).toDouble();
    GeomZ = settings.value("GeomZ", 0.05).toDouble();
    GeomForceScale = settings.value("GeomForceScale", 0.00001).toDouble();
    GeomForceRadius = settings.value("GeomForceRadius", 0.01).toDouble();
    StrapRadius = settings.value("StrapRadius", 0.005).toDouble();
    StrapForceRadius = settings.value("StrapForceRadius", 0.01).toDouble();
    StrapForceScale = settings.value("StrapForceScale", 0.00001).toDouble();
    StrapCylinderLength = settings.value("StrapCylinderLength", 0.04).toDouble();
    ReporterX = settings.value("ReporterX", 0.005).toDouble();
    ReporterY = settings.value("ReporterY", 0.005).toDouble();
    ReporterZ = settings.value("ReporterZ", 0.005).toDouble();
    DataTargetX = settings.value("DataTargetX", 0.005).toDouble();
    DataTargetY = settings.value("DataTargetY", 0.005).toDouble();
    DataTargetZ = settings.value("DataTargetZ", 0.005).toDouble();
    CursorRadius = settings.value("CursorRadius", 0.005).toDouble();
    Nudge = settings.value("Nudge", 0.005).toDouble();
    CameraFrontClip = settings.value("CameraFrontClip", 1).toDouble();
    CameraBackClip = settings.value("CameraBackClip", 100).toDouble();

    LastFileOpened = settings.value("LastFileOpened", "").toByteArray();
    GraphicsPath = settings.value("GraphicsPath", "").toByteArray();

    DisplayTracking = settings.value("DisplayTracking", false).toBool();
    DisplayOverlay = settings.value("DisplayOverlay", false).toBool();
    DisplayContactForces = settings.value("DisplayContactForces", false).toBool();
    DisplayMuscleForces = settings.value("DisplayMuscleForces", false).toBool();
    DisplayFramerate = settings.value("DisplayFramerate", false).toBool();
    DisplayActivation = settings.value("DisplayActivation", false).toBool();

    OpenCLUseOpenCL = settings.value("OpenCLUseOpenCL", false).toBool();
    OpenCLUseOpenCLLog = settings.value("OpenCLUseOpenCLLog", false).toBool();
    OpenCLDeviceDeviceNumber = settings.value("OpenCLDeviceDeviceNumber", 0).toInt();
    OpenCLTargetPlatform = settings.value("OpenCLTargetPlatform", 0).toInt();
    OpenCLDeviceType = settings.value("OpenCLDeviceType", "").toByteArray();

    MoviePath = settings.value("MoviePath", "Movie").toByteArray();
    MovieSkip = settings.value("MovieSkip", 100).toInt();
    MovieFormat = settings.value("MovieFormat", 0).toInt();
    QuicktimeFramerate = settings.value("QuicktimeFramerate", 25).toInt();
    OpenGLMultisample = settings.value("OpenGLMultisample", 8).toInt();

    DefaultTrackingOffset = settings.value("DefaultTrackingOffset", 0).toDouble();

    DefaultCameraDistance = settings.value("DefaultCameraDistance", 50).toDouble();
    DefaultCameraFoV = settings.value("DefaultCameraFoV", 5).toDouble();
    DefaultCameraCOIX = settings.value("DefaultCameraCOIX", 0).toDouble();
    DefaultCameraCOIY = settings.value("DefaultCameraCOIY", 0).toDouble();
    DefaultCameraCOIZ = settings.value("DefaultCameraCOIZ", 0).toDouble();

    DefaultCameraVecX = settings.value("DefaultCameraVecX", 0).toDouble();
    DefaultCameraVecY = settings.value("DefaultCameraVecY", 1).toDouble();
    DefaultCameraVecZ = settings.value("DefaultCameraVecZ", 0).toDouble();
    DefaultCameraUpX = settings.value("DefaultCameraUpX", 0).toDouble();
    DefaultCameraUpY = settings.value("DefaultCameraUpY", 0).toDouble();
    DefaultCameraUpZ = settings.value("DefaultCameraUpZ", 1).toDouble();

    // handle default Colors
    if (settings.contains("BackgroundColour") == false) settings.setValue("BackgroundColour", QColor(255, 255, 255, 255));
    if (settings.contains("EnvironmentColour") == false) settings.setValue("EnvironmentColour", QColor(0,255,255));
    if (settings.contains("BodyColour") == false) settings.setValue("BodyColour", QColor(0,255,0));
    if (settings.contains("JointColour") == false) settings.setValue("JointColour", QColor(0,0,255));
    if (settings.contains("GeomColour") == false) settings.setValue("GeomColour", QColor(255,255,0));
    if (settings.contains("GeomForceColour") == false) settings.setValue("GeomForceColour", QColor(255,255,0));
    if (settings.contains("StrapColour") == false) settings.setValue("StrapColour", QColor(255,0,0));
    if (settings.contains("StrapForceColour") == false) settings.setValue("StrapForceColour", QColor(255,0,0));
    if (settings.contains("StrapCylinderColour") == false) settings.setValue("StrapCylinderColour", QColor(255,0,255));
    if (settings.contains("ReporterColour") == false) settings.setValue("ReporterColour", QColor(255,128,0));
    if (settings.contains("DataTargetColour") == false) settings.setValue("DataTargetColour", QColor(255,0,128));

    BackgroundColour = settings.value("BackgroundColour").value<QColor>();
    EnvironmentColour = settings.value("EnvironmentColour").value<QColor>();
    BodyColour = settings.value("BodyColour").value<QColor>();
    JointColour = settings.value("JointColour").value<QColor>();
    GeomColour = settings.value("GeomColour").value<QColor>();
    GeomForceColour = settings.value("GeomForceColour").value<QColor>();
    StrapColour = settings.value("StrapColour").value<QColor>();
    StrapForceColour = settings.value("StrapForceColour").value<QColor>();
    StrapCylinderColour = settings.value("StrapCylinderColour").value<QColor>();
    ReporterColour = settings.value("ReporterColour").value<QColor>();
    DataTargetColour = settings.value("DataTargetColour").value<QColor>();
}

void Preferences::Write()
{
    QSettings settings(Organisation, Application);

    settings.setValue("MainWindowGeometry", MainWindowGeometry);
    settings.setValue("MainWindowState", MainWindowState);
    settings.setValue("SplitterState", SplitterState);
    settings.setValue("CameraDistance", CameraDistance);
    settings.setValue("CameraFoV", CameraFoV);
    settings.setValue("CameraCOIX", CameraCOIX);
    settings.setValue("CameraCOIY", CameraCOIY);
    settings.setValue("CameraCOIZ", CameraCOIZ);
    settings.setValue("CameraVecX", CameraVecX);
    settings.setValue("CameraVecY", CameraVecY);
    settings.setValue("CameraVecZ", CameraVecZ);
    settings.setValue("CameraUpX", CameraUpX);
    settings.setValue("CameraUpY", CameraUpY);
    settings.setValue("CameraUpZ", CameraUpZ);
    settings.setValue("TrackingOffset", TrackingOffset);
    settings.setValue("TrackingFlag", TrackingFlag);
    settings.setValue("YUp", YUp);
    settings.setValue("OrthographicFlag", OrthographicFlag);

    settings.setValue("FrameSkip", FrameSkip);

    settings.setValue("EnvironmentX", EnvironmentX);
    settings.setValue("EnvironmentY", EnvironmentY);
    settings.setValue("EnvironmentZ", EnvironmentZ);
    settings.setValue("BodyX", BodyX);
    settings.setValue("BodyY", BodyY);
    settings.setValue("BodyZ", BodyZ);
    settings.setValue("JointX", JointX);
    settings.setValue("JointY", JointY);
    settings.setValue("JointZ", JointZ);
    settings.setValue("GeomX", GeomX);
    settings.setValue("GeomY", GeomY);
    settings.setValue("GeomZ", GeomZ);
    settings.setValue("GeomForceScale", GeomForceScale);
    settings.setValue("GeomForceRadius", GeomForceRadius);
    settings.setValue("StrapRadius", StrapRadius);
    settings.setValue("StrapForceRadius", StrapForceRadius);
    settings.setValue("StrapForceScale", StrapForceScale);
    settings.setValue("StrapCylinderLength", StrapCylinderLength);
    settings.setValue("ReporterX", ReporterX);
    settings.setValue("ReporterY", ReporterY);
    settings.setValue("ReporterZ", ReporterZ);
    settings.setValue("DataTargetX", DataTargetX);
    settings.setValue("DataTargetY", DataTargetY);
    settings.setValue("DataTargetZ", DataTargetZ);
    settings.setValue("CursorRadius", CursorRadius);
    settings.setValue("Nudge", Nudge);
    settings.setValue("CameraFrontClip", CameraFrontClip);
    settings.setValue("CameraBackClip", CameraBackClip);

    settings.setValue("LastFileOpened", LastFileOpened);
    settings.setValue("GraphicsPath", GraphicsPath);

    settings.setValue("BackgroundColour", BackgroundColour);
    settings.setValue("EnvironmentColour", EnvironmentColour);
    settings.setValue("BodyColour", BodyColour);
    settings.setValue("JointColour", JointColour);
    settings.setValue("GeomColour", GeomColour);
    settings.setValue("GeomForceColour", GeomForceColour);
    settings.setValue("StrapColour", StrapColour);
    settings.setValue("StrapForceColour", StrapForceColour);
    settings.setValue("StrapCylinderColour", StrapCylinderColour);
    settings.setValue("ReporterColour", ReporterColour);
    settings.setValue("DataTargetColour", DataTargetColour);

    settings.setValue("DisplayTracking", DisplayTracking);
    settings.setValue("DisplayOverlay", DisplayOverlay);
    settings.setValue("DisplayContactForces", DisplayContactForces);
    settings.setValue("DisplayMuscleForces", DisplayMuscleForces);
    settings.setValue("DisplayFramerate", DisplayFramerate);
    settings.setValue("DisplayActivation", DisplayActivation);

    settings.setValue("OpenCLUseOpenCL", OpenCLUseOpenCL);
    settings.setValue("OpenCLUseOpenCLLog", OpenCLUseOpenCLLog);
    settings.setValue("OpenCLDeviceDeviceNumber", OpenCLDeviceDeviceNumber);
    settings.setValue("OpenCLTargetPlatform", OpenCLTargetPlatform);
    settings.setValue("OpenCLDeviceType", OpenCLDeviceType);

    settings.setValue("MoviePath", MoviePath);
    settings.setValue("MovieSkip", MovieSkip);
    settings.setValue("MovieFormat", MovieFormat);
    settings.setValue("QuicktimeFramerate", QuicktimeFramerate);
    settings.setValue("OpenGLMultisample", OpenGLMultisample);

    settings.setValue("DefaultTrackingOffset", DefaultTrackingOffset);

    settings.setValue("DefaultCameraDistance", DefaultCameraDistance);
    settings.setValue("DefaultCameraFoV", DefaultCameraFoV);
    settings.setValue("DefaultCameraCOIX", DefaultCameraCOIX);
    settings.setValue("DefaultCameraCOIY", DefaultCameraCOIY);
    settings.setValue("DefaultCameraCOIZ", DefaultCameraCOIZ);

    settings.setValue("DefaultCameraVecX", DefaultCameraVecX);
    settings.setValue("DefaultCameraVecY", DefaultCameraVecY);
    settings.setValue("DefaultCameraVecZ", DefaultCameraVecZ);
    settings.setValue("DefaultCameraUpX", DefaultCameraUpX);
    settings.setValue("DefaultCameraUpY", DefaultCameraUpY);
    settings.setValue("DefaultCameraUpZ", DefaultCameraUpZ);

    settings.sync();
}
