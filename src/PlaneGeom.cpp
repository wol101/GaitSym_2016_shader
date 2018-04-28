/*
 *  PlaneGeom.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 13/09/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include "PlaneGeom.h"
#include "Simulation.h"

#include <ode/ode.h>

#include <algorithm>

#ifdef USE_QT
#include "GLUtils.h"
#endif

// Simulation global
extern Simulation *gSimulation;

// The plane equation is:
// a * x + b * y + c * z = d
// The plane's normal vector is (a, b, c), and it must have length 1.

// Note: planes are non placable so do not try and place them!

PlaneGeom::PlaneGeom(dSpaceID space, double a, double b, double c, double d)
{
    // create the geom
    m_GeomID = dCreatePlane(space, a, b, c, d);
    dGeomSetData(m_GeomID, this);

#ifdef USE_QT
    //glGenTextures(1, &m_textureID);
    m_texture = 0;
    m_trackDepth = 0;
    m_lastDisplayTime = -1;
    m_nx = 0;
    m_ny = 0;
    Colour mappedColour;
    for (int i = 0; i < 256; i++)
    {
        GLUtils::SetColourFromMap(float(i) / 255.0, JetColourMap, &mappedColour);
        m_colourMap[i][0] = int(0.5 + mappedColour.r * 255.0); // red
        m_colourMap[i][1] = int(0.5 + mappedColour.g * 255.0); // green
        m_colourMap[i][2] = int(0.5 + mappedColour.b * 255.0); // blue
        m_colourMap[i][3] = int(0.5 + mappedColour.alpha * 255.0); // alpha
    }

    m_planeOrigin = pgd::Vector(a * d, b * d, c * d); // this is in world coordinates
    // now find a rotation that aligns the world Z axis with the plane normal
    m_planeOrientation = FindRotation(pgd::Vector(0, 0, 1), pgd::Vector(a, b, c));
    m_checkerboardLow = 0;
    m_checkerboardHigh = 0;
    m_trackDrawThreshold = 0;
#endif

}

PlaneGeom::~PlaneGeom()
{
#ifdef USE_QT
    //glDeleteTextures(1, &m_textureID);
    if (m_texture) delete m_texture;
    if (m_trackDepth) delete m_trackDepth;
#endif
}

#ifdef USE_QT
// this routine sets the limits and resolution of the coloured patch used for the footprint
void PlaneGeom::SetTrackPatch(double trackPatchStartX, double trackPatchStartY, double trackPatchEndX, double trackPatchEndY, double trackPatchResolutionX, double trackPatchResolutionY)
{
    m_trackPatchStartX = trackPatchStartX;
    m_trackPatchEndX = trackPatchEndX;
    m_trackPatchResolutionX = trackPatchResolutionX;
    m_trackPatchStartY = trackPatchStartY;
    m_trackPatchEndY = trackPatchEndY;
    m_trackPatchResolutionY = trackPatchResolutionY;

    m_nx = (int)(0.00001 + (m_trackPatchEndX - m_trackPatchStartX) / m_trackPatchResolutionX);
    m_ny = (int)(0.00001 + (m_trackPatchEndY - m_trackPatchStartY) / m_trackPatchResolutionY);
    int textureSize = m_nx * m_ny * 4; // RGBA format
    if (textureSize)
    {
        m_texture = new unsigned char[textureSize];
        std::fill_n(m_texture, textureSize, 0);
        m_trackDepth = new double[textureSize];
        if (m_checkerboardHigh == m_checkerboardLow)
        {
            std::fill_n(m_trackDepth, textureSize, m_checkerboardLow);
        }
        else
        {
            double *trackDepthPtr = m_trackDepth;
            for (int iy = 0; iy < m_ny; iy++)
            {
                for (int ix = 0; ix < m_nx; ix++)
                {
                    if (iy % 2)
                    {
                        if (ix %2) *trackDepthPtr++ = m_checkerboardHigh;
                        else *trackDepthPtr++ = m_checkerboardLow;
                    }
                    else
                    {
                        if (ix %2) *trackDepthPtr++ = m_checkerboardLow;
                        else *trackDepthPtr++ = m_checkerboardHigh;
                    }
                }
            }
        }
    }
}

// this routine adds a impulse to the plane impulse store to emulate the appearance of footprints
//.the values are in world coordinates
void PlaneGeom::AddImpulse(double x, double y, double z, double fx, double fy, double fz, double time)
{
    if (m_trackDepth == 0) return;
    pgd::Vector offsetVec =  pgd::Vector(x, y, z) - m_planeOrigin;
    pgd::Vector posOnPlane = QVRotate(m_planeOrientation, offsetVec);
    pgd::Vector force = QVRotate(m_planeOrientation, pgd::Vector(fx, fy, fz));
    pgd::Vector impulse = force * time;
    int ix = (int)((posOnPlane.x - m_trackPatchStartX) / m_trackPatchResolutionX);
    int iy = (int)((posOnPlane.y - m_trackPatchStartY) / m_trackPatchResolutionY);
    if (ix >= 0 && x < m_nx && iy >= 0 && y < m_ny)
    {
        double *ptr = m_trackDepth + iy * m_nx + ix;
        *ptr += impulse.z;
        // std::cerr << "ix = " << ix << " iy = " << iy << " Impulse = " << *ptr << "\n";
    }
}


void PlaneGeom::Draw()
{
    /* old style opengl
    if (m_Visible == false) return;
    if (m_trackDepth == 0 || m_texture == 0) return;

    if (m_lastDisplayTime != gSimulation->GetTime())
    {
        m_lastDisplayTime = gSimulation->GetTime();
        glBindTexture(GL_TEXTURE_2D, m_textureID);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

        // fill the texture values from the track depth map
        unsigned char *texturePtr = m_texture;
        int ix, iy;
        double *trackDepthPtr = m_trackDepth;
        double v;
        int idx;
        int trackDrawThresholdIdx = int(255.99999 * (m_trackDrawThreshold - m_lowRange) / (m_highRange - m_lowRange));
        for (iy = 0; iy < m_ny; iy++)
        {
            for (ix = 0; ix < m_nx; ix++)
            {
                v = (*trackDepthPtr++ - m_lowRange) / (m_highRange - m_lowRange);
                idx = (int)(255.99999 * v);
                if (idx < 0) idx = 0;
                else if (idx > 255) idx = 255;
                *texturePtr++ = m_colourMap[idx][0];
                *texturePtr++ = m_colourMap[idx][1];
                *texturePtr++ = m_colourMap[idx][2];
                if (idx <= trackDrawThresholdIdx) *texturePtr++ = m_colourMap[idx][3];
                else *texturePtr++ = 0;
            }
        }

        glTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA, m_nx, m_ny, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_texture); // glTexSubmImage2D may be quicker

    }

    GLfloat xmin = m_trackPatchStartX;
    GLfloat ymin = m_trackPatchStartY;
    GLfloat xmax = m_trackPatchEndX;
    GLfloat ymax = m_trackPatchEndY;

    glPushMatrix();

    glTranslatef(m_planeOrigin.x, m_planeOrigin.y, m_planeOrigin.z);
    glRotatef(pgd::RadiansToDegrees(2*acos(m_planeOrientation.n)), m_planeOrientation.v.x, m_planeOrientation.v.y, m_planeOrientation.v.z);

    glEnable(GL_TEXTURE_2D);
    glDisable(GL_CULL_FACE);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    glBindTexture(GL_TEXTURE_2D, m_textureID);
    glBegin (GL_QUADS);
    glTexCoord2f (0.0, 0.0);
    glVertex3f (xmin, ymin, 0.0);
    glTexCoord2f (1.0, 0.0);
    glVertex3f (xmax, ymin, 0.0);
    glTexCoord2f (1.0, 1.0);
    glVertex3f (xmax, ymax, 0.0);
    glTexCoord2f (0.0, 1.0);
    glVertex3f (xmin, ymax, 0.0);
    glEnd ();

    glEnable(GL_CULL_FACE);
    glDisable(GL_TEXTURE_2D);

    glPopMatrix();
    */
}
#endif
