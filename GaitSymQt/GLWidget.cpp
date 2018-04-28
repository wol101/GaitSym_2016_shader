#include "glwidget.h"
#include "trackball.h"
#include "Simulation.h"
#include "PGDMath.h"
#include "TIFFWrite.h"
#include "RayGeom.h"
#include "Contact.h"
#include "FacetedSphere.h"
#include "Body.h"
#include "dialoginterface.h"
#include "QTKitHelper.h"
#include "GLUtils.h"

#include <QtGui>
#include <QtOpenGL>
#include <QClipboard>
#include <QWheelEvent>
#include <QElapsedTimer>
#include <QFont>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <string>

// #define DRAW_LOGO // used for debugging

// Simulation global
extern Simulation *gSimulation;

// output globals
extern bool gDestinationOpenGL;
extern bool gDestinationPOVRay;
extern std::ofstream *gPOVRayFile;
extern bool gDestinationOBJFile;
extern std::ofstream *gOBJFile;
extern int gVertexOffset;

GLWidget::GLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    XYAspect = 1;
    cameraDistance = 50;
    frontClip = 1;
    backClip = 100;
    FOV = 5;
    COIx = COIy = COIz = 0;
    cameraVecX = 0;
    cameraVecY = 1;
    cameraVecZ = 0;
    upX = 0;
    upY = 0;
    upZ = 1;
    overlayFlag = false;
    movieFormat = GLWidget::Quicktime;
    yUp = false;
    m_orthographicProjection = true;
    frontClip = 0;
    backClip = 1;
    cursorRadius = 0.001;
    m3DCursorNudge = 0.001;
    cursorSphere = new FacetedSphere(cursorRadius, 4);
    cursorSphere->SetColour(1, 1, 1, 1);
    backRed = 0;
    backGreen = 0;
    backBlue = 0;
    backAlpha = 1;

    trackball = new Trackball();
    mTrackball = false;
    mPan = false;
    mZoom = false;

    qtMovie = 0;

    setCursor(Qt::CrossCursor);

    timer.start();
    displayFrameRate = false;

    bitmap = new QImage(256, 32, QImage::Format_Mono);
    painter = new QPainter(bitmap);
    font = new QFont("Courier", 18);
    openGLBitmap = new GLubyte[256 * 32 / 8];
    font->setStyleStrategy(QFont::StyleStrategy(QFont::PreferBitmap | QFont::NoAntialias | QFont::PreferQuality));
    painter->setFont(*font);
    painter->setPen(QPen(QColor(255, 255, 255), 1));
}

GLWidget::~GLWidget()
{
    cleanup();

    delete trackball;
    delete cursorSphere;
    delete [] openGLBitmap;
    delete font;
    delete painter;
    delete bitmap;
}

void GLWidget::cleanup()
{
    makeCurrent();
#ifdef DRAW_LOGO
    m_logoVbo.destroy();
#endif
    delete m_facetedObjectShader;
    m_facetedObjectShader = 0;
    delete m_fixedColourObjectShader;
    m_fixedColourObjectShader = 0;
    doneCurrent();
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(400, 400);
}

void GLWidget::initializeGL()
{
    // In this example the widget's corresponding top-level window can change
    // several times during the widget's lifetime. Whenever this happens, the
    // QOpenGLWidget's associated context is destroyed and a new one is created.
    // Therefore we have to be prepared to clean up the resources on the
    // aboutToBeDestroyed() signal, instead of the destructor. The emission of
    // the signal will be followed by an invocation of initializeGL() where we
    // can recreate all resources.
    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &GLWidget::cleanup);

    initializeOpenGLFunctions();

    QString versionString(QLatin1String(reinterpret_cast<const char*>(glGetString(GL_VERSION))));
    qDebug() << "Driver Version String:" << versionString;
    qDebug() << "Current Context:" << format();

    int openGLVersion = format().majorVersion() * 100 + format().minorVersion() * 10;
    if (openGLVersion < 330)
    {
        QString errorMessage = QString("This application requires OpenGL 3.3 or greater.\nCurrent version is %1.\nApplication will abort.").arg(versionString);
        QMessageBox::critical(this, tr("CloudDigitiser"), errorMessage);
        exit(EXIT_FAILURE);
    }

    // Create a vertex array object. In OpenGL ES 2.0 and OpenGL 2.x
    // implementations this is optional and support may not be present
    // at all. Nonetheless the below code works in all cases and makes
    // sure there is a VAO when one is needed.
    m_vao.create();
    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);

    glClearColor(backRed, backGreen, backBlue, backAlpha);

    m_facetedObjectShader = new QOpenGLShaderProgram;
    m_facetedObjectShader->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/opengl/vertex_shader.glsl");
    m_facetedObjectShader->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/opengl/fragment_shader.glsl");
    m_facetedObjectShader->bindAttributeLocation("vertex", 0);
    m_facetedObjectShader->bindAttributeLocation("vertexNormal", 1);
    m_facetedObjectShader->link();

    m_facetedObjectShader->bind();
    m_mvMatrixLoc = m_facetedObjectShader->uniformLocation("mvMatrix");
    m_mvpMatrixLoc = m_facetedObjectShader->uniformLocation("mvpMatrix");
    m_normalMatrixLoc = m_facetedObjectShader->uniformLocation("normalMatrix");
    m_lightPosLoc = m_facetedObjectShader->uniformLocation("lightPosition");
    m_diffuseLoc = m_facetedObjectShader->uniformLocation("diffuse");
    m_ambientLoc = m_facetedObjectShader->uniformLocation("ambient");
    m_specularLoc = m_facetedObjectShader->uniformLocation("specular");
    m_shininessLoc = m_facetedObjectShader->uniformLocation("shininess");

    m_facetedObjectShader->setUniformValue(m_lightPosLoc, QVector4D(100, 100, 100, 1) );
    m_facetedObjectShader->setUniformValue(m_diffuseLoc, QVector4D(0.5, 0.5, 0.5, 1.0) );  // diffuse
    m_facetedObjectShader->setUniformValue(m_ambientLoc, QVector4D(0.5, 0.5, 0.5, 1.0) );  // ambient
    m_facetedObjectShader->setUniformValue(m_specularLoc, QVector4D(0.5, 0.5, 0.5, 1.0) );  // specular reflectivity
    m_facetedObjectShader->setUniformValue(m_shininessLoc, 5.0f ); // specular shininess

    m_facetedObjectShader->release();

    m_fixedColourObjectShader = new QOpenGLShaderProgram;
    m_fixedColourObjectShader->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/opengl/vertex_shader_2.glsl");
    m_fixedColourObjectShader->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/opengl/fragment_shader_2.glsl");
    m_fixedColourObjectShader->bindAttributeLocation("vertex", 0);
    m_fixedColourObjectShader->bindAttributeLocation("vertexColor", 1);
    m_fixedColourObjectShader->link();

    m_fixedColourObjectShader->bind();
    m_mvpMatrixLoc2 = m_fixedColourObjectShader->uniformLocation("mvpMatrix");
    m_fixedColourObjectShader->release();

//    glGenBuffers(1, &m_LineBuffer);
//    glGenBuffers(1, &m_LineColourBuffer);

#ifdef DRAW_LOGO
    // Setup our vertex buffer object.
    m_logoVbo.create();
    m_logoVbo.bind();
    m_logoVbo.allocate(m_logo.constData(), m_logo.count() * sizeof(GLfloat));
    // Our camera never changes in this example.
    m_view.setToIdentity();
    m_view.translate(0, 0, -5);
#endif

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);

}

QMatrix4x4 GLWidget::view() const
{
    return m_view;
}

QMatrix4x4 GLWidget::proj() const
{
    return m_proj;
}

int GLWidget::normalMatrixLoc() const
{
    return m_normalMatrixLoc;
}

int GLWidget::mvMatrixLoc() const
{
    return m_mvMatrixLoc;
}

QOpenGLShaderProgram *GLWidget::facetedObjectShader() const
{
    return m_facetedObjectShader;
}

QOpenGLShaderProgram *GLWidget::fixedColourObjectShader() const
{
    return m_fixedColourObjectShader;
}

void GLWidget::paintGL()
{
    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);

    glClearColor(backRed, backGreen, backBlue, backAlpha);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // set the projection matrix
    m_proj.setToIdentity();
    if (m_orthographicProjection)
    {
        GLfloat viewSize = 0.5 * cameraDistance * sin(float(M_PI/180) * FOV);
        m_proj.ortho(-XYAspect * viewSize, XYAspect * viewSize, -viewSize, viewSize, frontClip, backClip); // multiply by orthographic projection matrix
    }
    else
    {
        m_proj.perspective(FOV, XYAspect, frontClip, backClip); // multiply by perspective projection
    }

    // set the view matrix
    m_view.setToIdentity();
    QVector3D eye(COIx - cameraVecX * cameraDistance, COIy - cameraVecY * cameraDistance, COIz - cameraVecZ * cameraDistance);
    QVector3D centre(COIx, COIy, COIz);
    QVector3D up(upX, upY, upZ);
    m_view.lookAt(eye, centre, up);


#ifdef DRAW_LOGO
    // Store the vertex attribute bindings for the program.
    m_logoVbo.bind();
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glEnableVertexAttribArray(0);
    f->glEnableVertexAttribArray(1);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), 0);
    f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));
    m_logoVbo.release();

    m_model.setToIdentity();
    m_facetedObjectShader->bind();
    m_facetedObjectShader->setUniformValue(m_mvMatrixLoc, m_view * m_model);
    m_facetedObjectShader->setUniformValue(m_mvpMatrixLoc, m_proj * m_view * m_model);
    QMatrix3x3 normalMatrix = m_model.normalMatrix();
    m_facetedObjectShader->setUniformValue(m_normalMatrixLoc, normalMatrix);

    glDrawArrays(GL_TRIANGLES, 0, m_logo.vertexCount());
    m_facetedObjectShader->release();
#endif

    // now draw things
    if (gSimulation)
    {
        StrokeFont modelStrokeFont;
        modelStrokeFont.setGlWidget(this);
        modelStrokeFont.setVpMatrix(m_proj * m_view);
        GLUtils::setStrokeFont(&modelStrokeFont);
        gSimulation->Draw(this);
        modelStrokeFont.Draw();
        // modelStrokeFont.Debug();
    }

    // the 3d cursor
    //cursorSphere->SetDisplayPosition(m3DCursor.x, m3DCursor.y, m3DCursor.z);
    //cursorSphere->setGlWidget(this);
    //cursorSphere->Draw();

    // any manipulation feedback?

    // raster mode positioning
    // with origin at top left
    glDisable(GL_DEPTH_TEST);
    StrokeFont strokeFont;
    const GLfloat threshold = 105.0 / 255.0;
    GLfloat backgroundDelta = (backRed * 0.299) + (backGreen * 0.587) + (backBlue * 0.114);
    if (backgroundDelta > threshold) strokeFont.SetRGBA(0, 0, 0, 1);
    else strokeFont.SetRGBA(1, 1, 1, 1);
    strokeFont.setGlWidget(this);
    QMatrix4x4 lineVP;
    lineVP.ortho(0, width(), 0, height(), -1, 1);
    strokeFont.setVpMatrix(lineVP);
    glLineWidth(2); // this doesn't seem to work on the Mac

    if (mZoom)
    {
        double centreX = double(width()) / 2;
        double centreY = double(height()) / 2;
        double xMouse = double(mMouseX) - centreX;
        double yMouse = centreY - double(mMouseY); // this is because the mouse Y axis is negative the OpenGL Y axis
        double radius = sqrt(xMouse * xMouse + yMouse * yMouse);
        strokeFont.AddCircle(centreX, centreY, 0, radius, 180);
    }

    if (mTrackball  && trackball->GetOutsideRadius())
    {
        double centreX = double(width()) / 2;
        double centreY = double(height()) / 2;
        double radius = trackball->GetTrackballRadius();
        strokeFont.AddCircle(centreX, centreY, 0, radius, 180);
    }

    qint64 msSinceLastPaint = timer.restart();
    double frameRate;
    if (msSinceLastPaint) frameRate = 1000.0 / double(msSinceLastPaint);
    else frameRate = 0;
    if (displayFrameRate)
    {
        QString framesPerSecond = QString("Framerate: %1").arg(frameRate, 6, 'f', 2);
        strokeFont.StrokeString(framesPerSecond.toLatin1(), framesPerSecond.size(), 20, 20, 10, 10, 0, 0, 0, 0);
    }

   strokeFont.Draw();
}

void GLWidget::resizeGL(int width, int height)
{
    //m_proj.setToIdentity();
    //m_proj.perspective(45.0f, GLfloat(width) / height, 0.01f, 100.0f);

    XYAspect = static_cast<GLfloat>(width) / static_cast<GLfloat>(height);
    // qDebug("width=%d height=%d XYAspect=%f\n", width, height, XYAspect);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    if (gSimulation == 0) return;

    mMouseX = event->pos().x();
    mMouseY = event->pos().y();

    mTrackball = false;
    if (event->buttons() & Qt::LeftButton)
    {
        if (event->modifiers() == Qt::NoModifier)
        {
            int trackballRadius;
            if (width() < height()) trackballRadius = width() / 2.2;
            else trackballRadius = height() / 2.2;
            mTrackballStartCameraVec = pgd::Vector(cameraVecX, cameraVecY, cameraVecZ);
            mTrackballStartUp = pgd::Vector(upX, upY, upZ);
            trackball->StartTrackball(event->pos().x(), event->pos().y(), width() / 2, height() / 2, trackballRadius, mTrackballStartUp, -mTrackballStartCameraVec);
            mTrackball = true;
            emit EmitStatusString(tr("Rotate"));
            update();
        }
        else if (event->modifiers() & Qt::ShiftModifier)
        {
            // create the collision ray

            // on high resolution (e.g.retina) displays the units of the viewport are device pixels whereas the units of event->pos() are scaled pixels
            // the following mapping should always give the right values for the UnProject matrix
            GLfloat winX, winY;
            winX = ((GLfloat)event->pos().x() / (GLfloat)width()) * 2 - 1;
            winY = -1 * (((GLfloat)event->pos().y() / (GLfloat)height()) * 2 - 1);

            // picking based on the near and far clipping planes
            QMatrix4x4 vpMatrix = m_proj * m_view; // model would be identity so mvpMatrix isn't needed
            QMatrix4x4 unprojectMatrix = vpMatrix.inverted();
            QVector4D screenPoint(winX, winY, -1, 1);
            QVector4D nearPoint4D = unprojectMatrix * screenPoint;
            screenPoint.setZ(+1);
            QVector4D farPoint4D = unprojectMatrix * screenPoint;

            QVector4D rayDirection4D = farPoint4D - nearPoint4D;
            rayDirection4D.normalize();
            double length = backClip;
            RayGeom rayGeom(0, length,
                            nearPoint4D.x(), nearPoint4D.y(), nearPoint4D.z(),
                            rayDirection4D.x(), rayDirection4D.y(), rayDirection4D.z());
            rayGeom.SetParams(0, 0, 0); // firstcontact=0, backfacecull=0, closestHit = 0

            // code for collision detection
            pgd::Vector cameraPosition = pgd::Vector(COIx, COIy, COIz) - cameraDistance * pgd::Vector(cameraVecX, cameraVecY, cameraVecZ);
            pgd::Vector closestContact;
            std::vector<Geom *> *pickGeomList = gSimulation->GetPickGeomList();
            std::map<std::string, Body *> *bodyList = gSimulation->GetBodyList();
            const int maxContacts = 128;
            dContactGeom contacts[maxContacts];
            int numCollisions;
            double distance2;
            pgd::Vector cameraRelVector;
            double minDistance2 = DBL_MAX;

            for (unsigned int j = 0; j < pickGeomList->size(); j++)
            {
                if ((*bodyList)[*(*pickGeomList)[j]->GetName()]->GetVisible())
                {

                    // std::cerr << *(*pickGeomList)[j]->GetName() << "\n";
                    numCollisions = dCollide (rayGeom.GetGeomID(), (*pickGeomList)[j]->GetGeomID(), maxContacts, contacts, sizeof(dContactGeom));
                    for (int i = 0; i < numCollisions; i++)
                    {
                        cameraRelVector = pgd::Vector(contacts[i].pos[0], contacts[i].pos[1], contacts[i].pos[2]) - cameraPosition;
                        distance2 = cameraRelVector.Magnitude2();
                        if (distance2 < minDistance2)
                        {
                            minDistance2 = distance2;
                            closestContact = pgd::Vector(contacts[i].pos[0], contacts[i].pos[1], contacts[i].pos[2]);
                        }
                    }
                }
            }

            if (minDistance2 < DBL_MAX)
            {
                Move3DCursor(closestContact.x, closestContact.y, closestContact.z);
                update();
            }

        }
    }
    else if (event->buttons() & Qt::RightButton)
    {
        mPan = true;

        GLfloat winX, winY;
        winX = ((GLfloat)event->pos().x() / (GLfloat)width()) * 2 - 1;
        winY = -1 * (((GLfloat)event->pos().y() / (GLfloat)height()) * 2 - 1);
        QVector4D screenPoint(winX, winY, 0, 1);
        QMatrix4x4 vpMatrix = m_proj * m_view; // model would be identity so mvpMatrix isn't needed
        unprojectPanMatrix = vpMatrix.inverted();
        panStartPoint4D = unprojectPanMatrix * screenPoint;
        panStartCOI = QVector3D(COIx, COIy, COIz);

//        // get pick in world coordinates
//        GLdouble objX, objY, objZ;
//        // get and store the matrices from the start of panning
//        glGetDoublev(GL_MODELVIEW_MATRIX,mPanModelMatrix);
//        glGetDoublev(GL_PROJECTION_MATRIX,mPanProjMatrix);
//        int viewport[4];
//        glGetIntegerv(GL_VIEWPORT,viewport);

//        GLfloat winX, winY, winZ;
//        winX = event->pos().x();
//        winY = event->pos().y();
//        winY = (GLfloat)viewport[3] - winY;
//        glReadPixels(winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
//        panStartZ = winZ; // winZ seems to get set to 1 of clicked on blank area
//        /*gluUnProject(winX, winY, winZ,
//                     mPanModelMatrix, mPanProjMatrix, viewport,
//                     &objX, &objY, &objZ );*/

//        mPanStartVec = pgd::Vector(objX, objY, objZ);
//        mPanStartCOI = pgd::Vector(COIx, COIy, COIz);

        emit EmitStatusString(tr("Pan"));
        update();
    }
    else if (event->buttons() & Qt::MidButton)
    {
        mZoom = true;
        // centred -1 to -1 normalised values
        double x = (double)(2 * event->pos().x()) / (double)width() - 1.0;
        double y = (double)(2 * (height() - event->pos().y())) / (double)height() - 1.0;
        mZoomDistance = sqrt(x * x + y * y);
        if (mZoomDistance < 0.05) mZoomDistance = 0.05;
        mZoomStartFOV = FOV;

        emit EmitStatusString(tr("Zoom"));
        update();
    }

}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    mMouseX = event->pos().x();
    mMouseY = event->pos().y();

    // std::cerr << mMouseX << "," << mMouseY << "\n";

    if (event->buttons() & Qt::LeftButton)
    {
        if (mTrackball)
        {
            pgd::Quaternion rotation;
            trackball->RollTrackballToClick(event->pos().x(), event->pos().y(), &rotation);
            pgd::Vector newCameraVec = pgd::QVRotate(~rotation, mTrackballStartCameraVec);
            cameraVecX = newCameraVec.x;
            cameraVecY = newCameraVec.y;
            cameraVecZ = newCameraVec.z;
            pgd::Vector newUp = pgd::QVRotate(~rotation, mTrackballStartUp);
            upX = newUp.x;
            upY = newUp.y;
            upZ = newUp.z;
            update();

            emit EmitStatusString(QString("Camera %1 %2 %3 Up %4 %5 %6").arg(cameraVecX).arg(cameraVecY).arg(cameraVecZ).arg(upX).arg(upY).arg(upZ));
        }
    }
    else if (event->buttons() & Qt::RightButton)
    {
        if (mPan)
        {
            GLfloat winX, winY;
            winX = ((GLfloat)event->pos().x() / (GLfloat)width()) * 2 - 1;
            winY = -1 * (((GLfloat)event->pos().y() / (GLfloat)height()) * 2 - 1);
            QVector4D screenPoint(winX, winY, 0, 1);
            QVector4D panCurrentPoint4D = unprojectPanMatrix * screenPoint;
            COIx = panStartCOI.x() - (panCurrentPoint4D.x() - panStartPoint4D.x());
            COIy = panStartCOI.y() - (panCurrentPoint4D.y() - panStartPoint4D.y());
            COIz = panStartCOI.z() - (panCurrentPoint4D.z() - panStartPoint4D.z());

            emit EmitStatusString(QString("COI %1 %2 %3").arg(COIx).arg(COIy).arg(COIz));
            emit EmitCOI(COIx, COIy, COIz);
            update();
        }
    }
    else if (event->buttons() & Qt::MidButton)
    {
        if (mZoom)
        {
            // centred -1 to -1 normalised values
            double x = (double)(2 * event->pos().x()) / (double)width() - 1.0;
            double y = (double)(2 * (height() - event->pos().y())) / (double)height() - 1.0;
            double zoomDistance = sqrt(x * x + y * y);
            FOV = mZoomStartFOV * mZoomDistance / zoomDistance;
            if (FOV > 170) FOV = 170;
            else if (FOV < 0.001) FOV = 0.001;
            update();

            emit EmitStatusString(QString("FOV %1").arg(FOV));
            emit EmitFoV(FOV);
        }
    }
}

void GLWidget::mouseReleaseEvent(QMouseEvent * /* event */)
{
    mTrackball = false;
    mPan = false;
    mZoom = false;
    update();
}

void GLWidget::wheelEvent(QWheelEvent * event)
{
    // assume each ratchet of the wheel gives a score of 120 (8 * 15 degrees)
    double sensitivity = 2400;
    double scale = 1.0 + double(event->delta()) / sensitivity;
    FOV *= scale;
    if (FOV > 170) FOV = 170;
    else if (FOV < 0.001) FOV = 0.001;
    update();
    emit EmitStatusString(QString("FOV %1").arg(FOV));
    emit EmitFoV(FOV);
}

void GLWidget::SetCameraRight()
{
    if (yUp)
    {
        cameraVecX = 0;
        cameraVecY = 0;
        cameraVecZ = -1;
        upX = 0;
        upY = 1;
        upZ = 0;
    }
    else
    {
        cameraVecX = 0;
        cameraVecY = 1;
        cameraVecZ = 0;
        upX = 0;
        upY = 0;
        upZ = 1;
    }
}

void GLWidget::SetCameraTop()
{
    if (yUp)
    {
        cameraVecX = 0;
        cameraVecY = 1;
        cameraVecZ = 0;
        upX = 0;
        upY = 0;
        upZ = 1;
    }
    else
    {
        cameraVecX = 0;
        cameraVecY = 0;
        cameraVecZ = -1;
        upX = 0;
        upY = 1;
        upZ = 0;
    }
}

void GLWidget::SetCameraFront()
{
    if (yUp)
    {
        cameraVecX = -1;
        cameraVecY = 0;
        cameraVecZ = 0;
        upX = 0;
        upY = 1;
        upZ = 0;
    }
    else
    {
        cameraVecX = -1;
        cameraVecY = 0;
        cameraVecZ = 0;
        upX = 0;
        upY = 0;
        upZ = 1;
    }
}

void GLWidget::SetCameraVec(double x, double y, double z)
{
    if (yUp)
    {
        cameraVecX = x;
        cameraVecY = z;
        cameraVecZ = -y;
        if (z > 0.999 || z < -0.999)
        {
            upX = 0;
            upY = 0;
            upZ = 1;
        }
        else
        {
            upX = 0;
            upY = 1;
            upZ = 0;
        }
   }
    else
    {
        cameraVecX = x;
        cameraVecY = y;
        cameraVecZ = z;
        if (z > 0.999 || z < -0.999)
        {
            upX = 0;
            upY = 1;
            upZ = 0;
        }
        else
        {
            upX = 0;
            upY = 0;
            upZ = 1;
        }
    }
    update(); // this one is needed because this routine is only called from the ViewControlWidget and it will generally cause a redraw
}

// write the current frame out to a file
int GLWidget::WriteFrame(const QString &filename, MovieFormat format)
{
    std::string pathname(filename.toUtf8());
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);

    switch(format)
    {
    case Quicktime:
        if (qtMovie)
        {
            unsigned char *rgb = new unsigned char[viewport[2] * viewport[3] * 3];
            unsigned char *rgb2 = new unsigned char[viewport[2] * viewport[3] * 3];
            glReadBuffer(GL_FRONT);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_RGB, GL_UNSIGNED_BYTE, rgb);
            // need to invert write order
            for (int i = 0; i < viewport[3]; i ++)
                memcpy(rgb2 + ((viewport[3] - i - 1) * viewport[2] * 3), rgb + (i * viewport[2] * 3), viewport[2] * 3);
            addImageToMovie(qtMovie, viewport[2], viewport[3], rgb2);
            delete [] rgb;
            delete [] rgb2;
            return 0;
        }
        break;


    case PPM:
        if (1)
        {
            unsigned char *rgb = new unsigned char[viewport[2] * viewport[3] * 3];
            glReadBuffer(GL_FRONT);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_RGB, GL_UNSIGNED_BYTE, rgb);
            std::string ppmpathname = pathname + ".ppm";
            WritePPM(ppmpathname.c_str(), viewport[2], viewport[3], (unsigned char *)rgb);
            delete [] rgb;
        }
        break;

    case TIFF:
        if (1)
        {
            unsigned char *rgb = new unsigned char[viewport[2] * viewport[3] * 3];
            glReadBuffer(GL_FRONT);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_RGB, GL_UNSIGNED_BYTE, rgb);
            std::string tiffpathname = pathname + ".tif";
            WriteTIFF(tiffpathname.c_str(), viewport[2], viewport[3], (unsigned char *)rgb);
            delete [] rgb;
        }
        break;

    case POVRay:
        if (1)
        {
            gDestinationOpenGL = false;
            gDestinationPOVRay = true;
            gDestinationOBJFile = false;
            std::string povpathname = pathname + ".pov";
            gPOVRayFile = new std::ofstream(povpathname.c_str());
            (*gPOVRayFile) << "#declare gCameraX = " << -cameraVecX * cameraDistance << " ;\n";
            (*gPOVRayFile) << "#declare gCameraY = " << -cameraVecY * cameraDistance << " ;\n";
            (*gPOVRayFile) << "#declare gCameraZ = " << -cameraVecZ * cameraDistance << " ;\n";
            (*gPOVRayFile) << "#declare gCOIx = " << COIx << " ;\n";
            (*gPOVRayFile) << "#declare gCOIy = " << COIy << " ;\n";
            (*gPOVRayFile) << "#declare gCOIz = " << COIz << " ;\n";
            (*gPOVRayFile) << "#declare gUpX = " << upX << " ;\n";
            (*gPOVRayFile) << "#declare gUpY = " << upY << " ;\n";
            (*gPOVRayFile) << "#declare gUpZ = " << upZ << " ;\n";
            (*gPOVRayFile) << "#declare gTime = " << gSimulation->GetTime() << " ;\n";

            (*gPOVRayFile) << "\n#include \"camera.pov\"\n\n";
            gSimulation->Draw(this);
            delete gPOVRayFile;
            gPOVRayFile = 0;
            gDestinationOpenGL = true;
            gDestinationPOVRay = false;
            gDestinationOBJFile = false;
        }
        break;

    case OBJ:
        if (1)
        {
            gVertexOffset = 0;
            gDestinationOpenGL = false;
            gDestinationPOVRay = false;
            gDestinationOBJFile = true;
            std::string objpathname = pathname + ".obj";
            gOBJFile = new std::ofstream(objpathname.c_str());
            gSimulation->Draw(this);
            delete gOBJFile;
            gOBJFile = 0;
            gDestinationOpenGL = true;
            gDestinationPOVRay = false;
            gDestinationOBJFile = false;
        }
        break;
    }

    return 0;
}

// write a PPM file (need to invert the y axis)
void GLWidget::WritePPM(const char *pathname, int width, int height, unsigned char *rgb)
{
    FILE *out;
    int i;

    out = fopen(pathname, "wb");

    // need to invert write order
    fprintf(out, "P6\n%d %d\n255\n", width, height);
    for (i = height - 1; i >= 0; i--)
        fwrite(rgb + (i * width * 3), width * 3, 1, out);

    fclose(out);
}

// write a TIFF file
void GLWidget::WriteTIFF(const char *pathname, int width, int height, unsigned char *rgb)
{
    TIFFWrite tiff;
    int i;

    tiff.initialiseImage(width, height, 72, 72, 3);
    // need to invert write order
    for (i = 0; i < height; i ++)
        tiff.copyRow(height - i - 1, rgb + (i * width * 3));

    tiff.writeToFile((char *)pathname);
}

// handle key presses
void GLWidget::HandleKeyPressEvent(QKeyEvent *e)
{
    switch( e->key() )
    {

        // X, Y and Z move the cursor
    case Qt::Key_X:
        if (e->modifiers() == Qt::NoModifier)
            m3DCursor.x += m3DCursorNudge;
        else
            m3DCursor.x -= m3DCursorNudge;
        Move3DCursor(m3DCursor.x, m3DCursor.y, m3DCursor.z);
        update();
        break;

    case Qt::Key_Y:
        if (e->modifiers() == Qt::NoModifier)
            m3DCursor.y += m3DCursorNudge;
        else
            m3DCursor.y -= m3DCursorNudge;
        Move3DCursor(m3DCursor.x, m3DCursor.y, m3DCursor.z);
        update();
        break;

    case Qt::Key_Z:
        if (e->modifiers() == Qt::NoModifier)
            m3DCursor.z += m3DCursorNudge;
        else
            m3DCursor.z -= m3DCursorNudge;
        Move3DCursor(m3DCursor.x, m3DCursor.y, m3DCursor.z);
        update();
        break;

        // S snaps the cursor to the nearest whole number multiple of the nudge value
    case Qt::Key_S:
        m3DCursor.x = round(m3DCursor.x / m3DCursorNudge) * m3DCursorNudge;
        m3DCursor.y = round(m3DCursor.y / m3DCursorNudge) * m3DCursorNudge;
        m3DCursor.z = round(m3DCursor.z / m3DCursorNudge) * m3DCursorNudge;
        Move3DCursor(m3DCursor.x, m3DCursor.y, m3DCursor.z);
        update();
        break;
    }
}

// set the 3D cursor position
void GLWidget::Move3DCursor(double x, double y, double z)
{
    m3DCursor = pgd::Vector(x, y, z);
    QClipboard *clipboard = QApplication::clipboard();
    clipboard->setText(QString("%1\t%2\t%3").arg(m3DCursor.x).arg(m3DCursor.y).arg(m3DCursor.z), QClipboard::Clipboard);
    emit EmitStatusString(QString("3D Cursor %1\t%2\t%3").arg(m3DCursor.x).arg(m3DCursor.y).arg(m3DCursor.z));
}

int GLWidget::mvpMatrixLoc2() const
{
    return m_mvpMatrixLoc2;
}

int GLWidget::shininessLoc() const
{
    return m_shininessLoc;
}

int GLWidget::specularLoc() const
{
    return m_specularLoc;
}

int GLWidget::ambientLoc() const
{
    return m_ambientLoc;
}

int GLWidget::diffuseLoc() const
{
    return m_diffuseLoc;
}

int GLWidget::mvpMatrixLoc() const
{
    return m_mvpMatrixLoc;
}

void GLWidget::Set3DCursorRadius(double v)
{
    if (v >= 0)
    {
        cursorRadius = v;
        delete cursorSphere;
        cursorSphere = new FacetedSphere(cursorRadius, 4);
        cursorSphere->SetColour(1, 1, 1, 1);
    }
}


