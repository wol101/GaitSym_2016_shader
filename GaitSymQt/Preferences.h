/*
 *  Preferences.h
 *  GaitSym
 *
 *  Created by Bill Sellers on 12/06/2014.
 *  Copyright 2014 Bill Sellers. All rights reserved.
 *
 */

#ifndef PREFERENCES_H
#define PREFERENCES_H

#include <QString>
#include <QByteArray>
#include <QSize>
#include <QPoint>
#include <QColor>

class Preferences
{
public:
    Preferences();

    void Read();
    void Write();

    QString Organisation; // organisation string - used for file names
    QString Application; // application name - used for file names
    QByteArray MainWindowGeometry; // window size and position information at exit
    QByteArray MainWindowState; // window state at exit
    QByteArray SplitterState; // splitter state at exit
    double CameraDistance;
    double CameraFoV;
    double CameraCOIX;
    double CameraCOIY;
    double CameraCOIZ;
    double CameraVecX;
    double CameraVecY;
    double CameraVecZ;
    double CameraUpX;
    double CameraUpY;
    double CameraUpZ;
    double TrackingOffset;
    bool TrackingFlag;
    bool YUp;
    bool OrthographicFlag;
    QColor BackgroundColour;
    QColor EnvironmentColour;
    QColor BodyColour;
    QColor JointColour;
    QColor GeomColour;
    QColor GeomForceColour;
    QColor StrapColour;
    QColor StrapForceColour;
    QColor StrapCylinderColour;
    QColor ReporterColour;
    QColor DataTargetColour;
    double EnvironmentX;
    double EnvironmentY;
    double EnvironmentZ;
    double BodyX;
    double BodyY;
    double BodyZ;
    double JointX;
    double JointY;
    double JointZ;
    double GeomX;
    double GeomY;
    double GeomZ;
    double GeomForceScale;
    double GeomForceRadius;
    double StrapRadius;
    double StrapForceRadius;
    double StrapForceScale;
    double StrapCylinderLength;
    double ReporterX;
    double ReporterY;
    double ReporterZ;
    double DataTargetX;
    double DataTargetY;
    double DataTargetZ;
    double CursorRadius;
    double Nudge;
    double CameraFrontClip;
    double CameraBackClip;
    int FrameSkip; // simulation steps per display frame
    QString LastFileOpened; // path of last loaded file
    QString GraphicsPath;
    bool DisplayTracking;
    bool DisplayOverlay;
    bool DisplayContactForces;
    bool DisplayMuscleForces;
    bool DisplayFramerate;
    bool DisplayActivation;
    QString MoviePath;
    int MovieSkip;
    int MovieFormat;
    int QuicktimeFramerate;
    int OpenGLMultisample;
    bool OpenCLUseOpenCL;
    bool OpenCLUseOpenCLLog;
    int OpenCLDeviceDeviceNumber;
    int OpenCLTargetPlatform;
    QString OpenCLDeviceType;
    double DefaultTrackingOffset;
    double DefaultCameraDistance;
    double DefaultCameraFoV;
    double DefaultCameraCOIX;
    double DefaultCameraCOIY;
    double DefaultCameraCOIZ;
    double DefaultCameraVecX;
    double DefaultCameraVecY;
    double DefaultCameraVecZ;
    double DefaultCameraUpX;
    double DefaultCameraUpY;
    double DefaultCameraUpZ;

};

#endif // PREFERENCES_H
