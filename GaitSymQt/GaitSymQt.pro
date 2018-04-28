# -------------------------------------------------
# Project created by QtCreator 2009-07-04T17:08:56
# -------------------------------------------------
VERSION = 2016
AUTHOR = "Bill Sellers 2016"
macx {
    DEFINES += \
        USE_QT \
        dDOUBLE \
#        USE_OPENCL \
        USE_LIBTIFF USE_TIFF_LZW USE_PCA USE_CBLAS
    INCLUDEPATH += ../src \
        /usr/include/libxml2 \
        ${HOME}/Unix/include \
        /System/Library/Frameworks/OpenCL.framework/Versions/A/Headers
    LIBS += -lxml2 \
        ${HOME}/Unix/lib/libode.a ${HOME}/Unix/lib/libtiff.a ${HOME}/Unix/lib/libANN.a \
#        -framework OpenCL \
        -framework QTKit \
        -framework Cocoa \
        -framework Accelerate
    HEADERS += QTKitHelper.h
    OBJECTIVE_SOURCES += QTKitHelper.mm
    QMAKE_CXXFLAGS += -std=c++11
    QMAKE_MACOSX_DEPLOYMENT_TARGET = 10.10
    QMAKE_MAC_SDK = macosx10.11

    # Application signature (4 characters).
    SIGNATURE           =   "aSl+"
    # Adds the Mac icons to the application bundle.
    ICON                = GaitSymQt.icns
    # Copy in a customised Info.plist
    QMAKE_INFO_PLIST    = plist/Info.plist

    PkgInfo.target      =   PkgInfo
    PkgInfo.depends     =   $${TARGET}.app/Contents/PkgInfo
    PkgInfo.commands    =   @$(DEL_FILE) $$PkgInfo.depends$$escape_expand(\n\t) \
                            @echo "APPL$$SIGNATURE" > $$PkgInfo.depends
    QMAKE_EXTRA_TARGETS +=  PkgInfo
    PRE_TARGETDEPS      +=  $$PkgInfo.target

    CONFIG(debug, debug|release) {
        message(Debug build)
        COMPUTERNAME = $$system(scutil --get ComputerName)
        contains(COMPUTERNAME, H14-Mac-Pro) { DEFINES += EXPERIMENTAL }
        contains(COMPUTERNAME, GraphiteG4) { DEFINES += EXPERIMENTAL }
        contains(COMPUTERNAME, WIS-MacBookAir) { DEFINES += EXPERIMENTAL }
    }
    else {
        message(Release build)
    }
}
else:win32 {
    RC_FILE = app.rc
    DEFINES += USE_OPENGL \
        USE_QT \
        dDOUBLE \
        MALLOC_H_NEEDED \
        BYTE_ORDER=LITTLE_ENDIAN \
        NEED_BCOPY \
        LIBXML_STATIC \
        _CRT_SECURE_NO_WARNINGS
    INCLUDEPATH += ../src \
        c:/Users/wis/Documents/Unix/include \
        c:/Users/wis/Documents/Unix/include/libxml2
    LIBS += c:/Users/wis/Documents/Unix/lib/libxml2_a.lib \
        c:/Users/wis/Documents/Unix/lib/ode.lib \
        -L"C:\Program Files (x86)\Windows Kits\8.1\Lib\winv6.3\um\x64" \
        -L"C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\lib\amd64" \
        wsock32.lib ws2_32.lib

}
else:unix {
#    ICON = GaitSymQt.icns
    DEFINES += USE_OPENGL \
        USE_QT \
        dDOUBLE \
#        USE_LIBTIFF USE_TIFF_LZW \
        GL_GLEXT_PROTOTYPES
    INCLUDEPATH += ../src \
        /usr/include/libxml2 \
        ${HOME}/Unix/include \
        /usr/include/GL
#    LIBS += -lxml2 -ltiff -lGLU \
    LIBS += -lxml2 -lGLU \
        ${HOME}/Unix/lib/libode.a
}

QMAKE_CXXFLAGS_RELEASE += -O0 \ #turned off optimisation because of the sincos undefined symbol error
    -ffast-math
OBJECTS_DIR = obj
QT += opengl
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
TARGET = GaitSymQt
TEMPLATE = app
SOURCES += \
    main.cpp \
    AboutDialog.cpp \
    CustomFileDialogs.cpp \
    DialogInterface.cpp \
    DialogOutputSelect.cpp \
    DialogPreferences.cpp \
    DoubleSpinbox.cpp \
    GLWidget.cpp \
    LineEdit.cpp \
    MainWindow.cpp \
    Preferences.cpp \
    SpinBox.cpp \
    TrackBall.cpp \
    ViewControlWidget.cpp \
    ../src/Util.cpp \
    ../src/UGMMuscle.cpp \
    ../src/UDP.cpp \
    ../src/TwoPointStrap.cpp \
    ../src/TIFFWrite.cpp \
    ../src/ThreePointStrap.cpp \
    ../src/TCP.cpp \
    ../src/Strap.cpp \
    ../src/StepDriver.cpp \
    ../src/SphereGeom.cpp \
    ../src/Simulation.cpp \
    ../src/PlaneGeom.cpp \
    ../src/ObjectiveMain.cpp \
    ../src/NPointStrap.cpp \
    ../src/NamedObject.cpp \
    ../src/Muscle.cpp \
    ../src/MAMuscleExtended.cpp \
    ../src/MAMuscleComplete.cpp \
    ../src/MAMuscle.cpp \
    ../src/Joint.cpp \
    ../src/HingeJoint.cpp \
    ../src/GLUtils.cpp \
    ../src/Geom.cpp \
    ../src/FloatingHingeJoint.cpp \
    ../src/FixedJoint.cpp \
    ../src/fec.cpp \
    ../src/FacetedSphere.cpp \
    ../src/FacetedPolyline.cpp \
    ../src/FacetedObject.cpp \
    ../src/FacetedConicSegment.cpp \
    ../src/Face.cpp \
    ../src/ErrorHandler.cpp \
    ../src/Environment.cpp \
    ../src/DataTarget.cpp \
    ../src/DataFile.cpp \
    ../src/DampedSpringMuscle.cpp \
    ../src/CylinderWrapStrap.cpp \
    ../src/CyclicDriver.cpp \
    ../src/Contact.cpp \
    ../src/CappedCylinderGeom.cpp \
    ../src/Body.cpp \
    ../src/BallJoint.cpp \
    ../src/DataTargetScalar.cpp \
    ../src/DataTargetQuaternion.cpp \
    ../src/DataTargetVector.cpp \
    ../src/Driver.cpp \
    ../src/TrimeshGeom.cpp \
    ../src/RayGeom.cpp \
    ../src/Reporter.cpp \
    ../src/TorqueReporter.cpp \
    ../src/OpenCLRoutines.cpp \
    ../src/ExpressionParser.cpp \
    ../src/XMLConverter.cpp \
    ../src/ExpressionVec.cpp \
    ../src/ExpressionVar.cpp \
    ../src/ExpressionRef.cpp \
    ../src/ExpressionMat.cpp \
    ../src/ExpressionFunTransform3D.cpp \
    ../src/ExpressionFolder.cpp \
    ../src/PositionReporter.cpp \
    ../src/Marker.cpp \
    ../src/UniversalJoint.cpp \
    ../src/AMotorJoint.cpp \
    ../src/FacetedCappedCylinder.cpp \
    ../src/PIDMuscleLength.cpp \
    ../src/Drivable.cpp \
    ../src/Controller.cpp \
    ../src/SwingClearanceAbortReporter.cpp \
    ../src/TwoCylinderWrapStrap.cpp \
    ../src/SliderJoint.cpp \
    ../src/BoxGeom.cpp \
    ../src/FacetedBox.cpp \
    ../src/BoxCarDriver.cpp \
    ../src/StackedBoxCarDriver.cpp \
    ../src/PIDTargetMatch.cpp \
    ../src/Warehouse.cpp \
    ../src/PCA.cpp \
    ../src/FixedDriver.cpp \
    ../src/StrokeFont.cpp \
    Logo.cpp
HEADERS += \
    AboutDialog.h \
    CustomFileDialogs.h \
    DialogInterface.h \
    DialogOutputSelect.h \
    DialogPreferences.h \
    DoubleSpinbox.h \
    GLWidget.h \
    LineEdit.h \
    MainWindow.h \
    Preferences.h \
    QTKitHelper.h \
    SpinBox.h \
    TrackBall.h \
    ViewControlWidget.h \
    ../src/Util.h \
    ../src/UGMMuscle.h \
    ../src/UDP.h \
    ../src/TwoPointStrap.h \
    ../src/TIFFWrite.h \
    ../src/ThreePointStrap.h \
    ../src/TCP.h \
    ../src/Strap.h \
    ../src/StepDriver.h \
    ../src/SphereGeom.h \
    ../src/SocketMessages.h \
    ../src/Simulation.h \
    ../src/SimpleStrap.h \
    ../src/PlaneGeom.h \
    ../src/PGDMath.h \
    ../src/NPointStrap.h \
    ../src/NamedObject.h \
    ../src/Muscle.h \
    ../src/MPIStuff.h \
    ../src/MAMuscleExtended.h \
    ../src/MAMuscleComplete.h \
    ../src/MAMuscle.h \
    ../src/Joint.h \
    ../src/HingeJoint.h \
    ../src/GLUtils.h \
    ../src/Geom.h \
    ../src/FloatingHingeJoint.h \
    ../src/FixedJoint.h \
    ../src/fec.h \
    ../src/FacetedSphere.h \
    ../src/FacetedPolyline.h \
    ../src/FacetedObject.h \
    ../src/FacetedConicSegment.h \
    ../src/Face.h \
    ../src/ErrorHandler.h \
    ../src/Environment.h \
    ../src/Driver.h \
    ../src/DebugControl.h \
    ../src/DataTarget.h \
    ../src/DataFile.h \
    ../src/DampedSpringMuscle.h \
    ../src/CylinderWrapStrap.h \
    ../src/CyclicDriver.h \
    ../src/Contact.h \
    ../src/CappedCylinderGeom.h \
    ../src/Body.h \
    ../src/BallJoint.h \
    ../src/ObjectiveMain.h \
    ../src/DataTargetScalar.h \
    ../src/DataTargetQuaternion.h \
    ../src/DataTargetVector.h \
    ../src/TrimeshGeom.h \
    ../src/RayGeom.h \
    ../src/Reporter.h \
    ../src/TorqueReporter.h \
    ../src/OpenCLRoutines.h \
    ../src/ExpressionParser.h \
    ../src/XMLConverter.h \
    ../src/ExpressionVec.h \
    ../src/ExpressionVar.h \
    ../src/ExpressionRef.h \
    ../src/ExpressionMat.h \
    ../src/ExpressionFunTransform3D.h \
    ../src/ExpressionFolder.h \
    ../src/PositionReporter.h \
    ../src/Marker.h \
    ../src/UniversalJoint.h \
    ../src/AMotorJoint.h \
    ../src/FacetedCappedCylinder.h \
    ../src/PIDMuscleLength.h \
    ../src/Drivable.h \
    ../src/Controller.h \
    ../src/SwingClearanceAbortReporter.h \
    ../src/TwoCylinderWrapStrap.h \
    ../src/SliderJoint.h \
    ../src/BoxGeom.h \
    ../src/FacetedBox.h \
    ../src/BoxCarDriver.h \
    ../src/StackedBoxCarDriver.h \
    ../src/PIDTargetMatch.h \
    ../src/Warehouse.h \
    ../src/PCA.h \
    ../src/FixedDriver.h \
    ../src/StrokeFont.h \
    Logo.h
FORMS += \
    AboutDialog.ui \
    DialogInterface.ui \
    DialogOutputSelect.ui \
    DialogPreferences.ui \
    MainWindow.ui

RESOURCES += resources.qrc

OTHER_FILES += \
    plist/Info.plist \
    app.rc \
    Icon.ico




