#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "PGDMath.h"
#include "QTKitHelper.h"
#include "StrokeFont.h"
#include "Logo.h"

#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>

class Trackball;
class RayGeom;
class FacetedSphere;

struct Light
{
    GLfloat ambient[4];
    GLfloat diffuse[4];
    GLfloat specular[4];
    GLfloat position[4];

    void SetAmbient(GLfloat r, GLfloat g, GLfloat b, GLfloat a) { ambient[0] = r; ambient[1] = g; ambient[2] = b; ambient[3] = a; }
    void SetDiffuse(GLfloat r, GLfloat g, GLfloat b, GLfloat a) { diffuse[0] = r; diffuse[1] = g; diffuse[2] = b; diffuse[3] = a; }
    void SetSpecular(GLfloat r, GLfloat g, GLfloat b, GLfloat a) { specular[0] = r; specular[1] = g; specular[2] = b; specular[3] = a; }
    void SetPosition(GLfloat x, GLfloat y, GLfloat z, GLfloat a) { position[0] = x; position[1] = y; position[2] = z; position[3] = a; }
};

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = 0);
    ~GLWidget();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

    enum MovieFormat {TIFF, PPM, POVRay, OBJ, Quicktime};

    void SetCameraRight();
    void SetCameraTop();
    void SetCameraFront();
    void SetCameraDistance(GLfloat v) { if (cameraDistance != v) { cameraDistance = v; } }
    void SetCameraFoV(GLfloat v) { if (FOV != v) { FOV = v; } }
    void SetCameraCOIX(GLfloat v) { if (COIx != v) { COIx = v; } }
    void SetCameraCOIY(GLfloat v) { if (COIy != v) { COIy = v; } }
    void SetCameraCOIZ(GLfloat v) { if (COIz != v) { COIz = v; } }
    void SetCameraFrontClip(GLfloat v) { if (frontClip != v) { frontClip = v; } }
    void SetCameraBackClip(GLfloat v) { if (backClip != v) { backClip = v; } }
    void SetOverlay(bool v) { if (overlayFlag != v) { overlayFlag = v; } }
    void SetMovieFormat(MovieFormat v) { movieFormat = v; }
    void SetYUp(bool v) { yUp = v; }
    void Set3DCursorRadius(double v);
    void Set3DCursorNudge(double v) { if (v >= 0) { m3DCursorNudge = v; } }
    int WriteFrame(const QString &filename, MovieFormat format);
   void SetQTMovie(MyMovieHandle movie) { qtMovie = movie; }
    void SetDisplayFramerate(bool v) { displayFrameRate = v; }
    void HandleKeyPressEvent(QKeyEvent *event);

    MovieFormat GetMovieFormat() { return movieFormat; }
    MyMovieHandle GetQTMovie() { return qtMovie; }


    // these ones are really only used for save and load view since they can only be manipulated within the widget
    void GetCameraVec(GLfloat *x, GLfloat *y, GLfloat *z) { *x = cameraVecX; *y = cameraVecY; *z = cameraVecZ; }
    void GetCameraUp(GLfloat *x, GLfloat *y, GLfloat *z) { *x = upX; *y = upY; *z = upZ; }
    void SetCameraVec(GLfloat x, GLfloat y, GLfloat z) { cameraVecX = x; cameraVecY = y; cameraVecZ = z; }
    void SetCameraUp(GLfloat x, GLfloat y, GLfloat z) { upX = x; upY = y; upZ = z; }
    GLfloat GetCameraDistance() { return cameraDistance; }
    GLfloat GetCameraFoV() { return FOV; }
    GLfloat GetCameraCOIX() { return COIx; }
    GLfloat GetCameraCOIY() { return COIy; }
    GLfloat GetCameraCOIZ() { return COIz; }

    QOpenGLShaderProgram *facetedObjectShader() const;
    QOpenGLShaderProgram *fixedColourObjectShader() const;
    int mvMatrixLoc() const;
    int mvpMatrixLoc() const;
    int normalMatrixLoc() const;
    QMatrix4x4 proj() const;
    QMatrix4x4 view() const;
    int diffuseLoc() const;
    int ambientLoc() const;
    int specularLoc() const;
    int shininessLoc() const;
    int mvpMatrixLoc2() const;

public slots:
    void SetCameraVec(double x, double y, double z);
    void SetBackground(float red, float green, float blue, float alpha) { backRed = red; backGreen = green; backBlue = blue; backAlpha = alpha; }
    void cleanup();

signals:
     void EmitStatusString(QString s);
     void EmitCOI(double x, double y, double z);
     void EmitFoV(double v);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent * event);

    void WritePPM(const char *pathname, int width, int height, unsigned char *rgb);
    void WriteTIFF(const char *pathname, int width, int height, unsigned char *rgb);
    void Move3DCursor(double x, double y, double z);

    void SetupLights();

private:

    GLfloat XYAspect;
    GLfloat backRed, backGreen, backBlue, backAlpha;
    GLfloat cameraDistance;
    GLfloat FOV;
    GLfloat cameraVecX, cameraVecY, cameraVecZ;
    GLfloat COIx, COIy, COIz;
    GLfloat upX, upY, upZ;
    GLfloat frontClip;
    GLfloat backClip;

    bool overlayFlag;
    MovieFormat movieFormat;
    bool yUp;
    bool m_orthographicProjection;

    Trackball *trackball;
    bool mTrackball;
    pgd::Vector mTrackballStartCameraVec;
    pgd::Vector mTrackballStartUp;

    bool mPan;
    QMatrix4x4 unprojectPanMatrix;
    QVector4D panStartPoint4D;
    QVector3D panStartCOI;
//    pgd::Vector mPanStartVec;
//    pgd::Vector mPanStartCOI;
//    QMatrix4x4 mPanModelMatrix;
//    QMatrix4x4 mPanProjMatrix;
//    GLfloat panStartZ;

    bool mZoom;
    double mZoomDistance;
    double mZoomStartFOV;

    int mMouseX;
    int mMouseY;

    pgd::Vector m3DCursor;
    double cursorRadius;
    double m3DCursorNudge;
    FacetedSphere *cursorSphere;

    //RayGeom *mRay;

    MyMovieHandle qtMovie;

    QElapsedTimer timer;
    bool displayFrameRate;
    QImage *bitmap;
    QPainter *painter;
    QFont *font;
    GLubyte *openGLBitmap;

 //   StrokeFont m_StrokeFont;
//    GLuint m_LineBuffer;
//    GLuint m_LineColourBuffer;

#ifdef DRAW_LOGO
    Logo m_logo;
    QOpenGLBuffer m_logoVbo;
#endif
    QOpenGLVertexArrayObject m_vao;
    QOpenGLShaderProgram *m_facetedObjectShader;
    QOpenGLShaderProgram *m_fixedColourObjectShader;
    int m_mvMatrixLoc;
    int m_mvpMatrixLoc;
    int m_normalMatrixLoc;
    int m_lightPosLoc;
    int m_diffuseLoc;
    int m_ambientLoc;
    int m_specularLoc;
    int m_shininessLoc;
    int m_mvpMatrixLoc2;
    QMatrix4x4 m_proj;
    QMatrix4x4 m_view;
    QMatrix4x4 m_model;


};

#endif // GLWIDGET_H
