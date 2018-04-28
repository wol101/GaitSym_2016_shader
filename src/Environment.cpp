/*
 *  Environment.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat Mar 26 2005.
 *  Copyright (c) 2005 Bill Sellers. All rights reserved.
 *
 */


#include <ode/ode.h>

#ifdef USE_QT
#include "GLUtils.h"
#include "GLUtils.h"

// ruler definition
static float gRulerMin = -500;
static float gRulerMax = 500;
static float gRulerTextSize = 0.1;
static float gRulerTextInterval = 1.0;
static float gRulerTickSize = 0.05;
static float gRulerTickInterval = 0.5;

#endif

#include "Environment.h"
#include "Geom.h"


Environment::Environment()
{
}

Environment::~Environment()
{
    std::vector<Geom *>::const_iterator iter1;
    for (iter1=m_GeomList.begin(); iter1 != m_GeomList.end(); iter1++)
        delete *iter1;
}

void Environment::AddGeom(Geom *geom)
{
    m_GeomList.push_back(geom);
}

#ifdef USE_QT
void Environment::Draw()
{
    if (m_Visible == false) return;

    // draw origin axes
    GLUtils::DrawAxes(m_AxisSize[0], m_AxisSize[1], m_AxisSize[2]);

    // draw ruler
    GLUtils::SetDrawColour(m_Colour.r, m_Colour.g, m_Colour.b, m_Colour.alpha);
    char buffer[256];
    float v;
    float rotation[12];
    float cosa = 0; // angle = +90 degrees
    float sina = 1; // angle = +90 degrees
    rotation[0] = 1; rotation[1] = 0;    rotation[2] = 0;
    rotation[4] = 0; rotation[5] = cosa; rotation[6] = -sina;
    rotation[8] = 0; rotation[9] = sina; rotation[10] = cosa;
    for (v = gRulerMin; v <= gRulerMax; v += gRulerTextInterval)
    {
        sprintf(buffer, "%.1f", v);
        GLUtils::OutputText(v, -2 * gRulerTextSize, 0, buffer, gRulerTextSize, rotation, 0);
    }

    for (v = gRulerMin; v <= gRulerMax; v += gRulerTickInterval)
    {
        GLUtils::DrawLine(v, 0, 0, v, 0, -gRulerTickSize);
    }
    // draw as 2 lines so we don't overwite the axis
    GLUtils::DrawLine(gRulerMin, 0, 0, 0, 0, 0);
    GLUtils::DrawLine(m_AxisSize[0], 0, 0, gRulerMax, 0, 0);

    // draw the geoms afterwards because they could use transparency
    for (unsigned int i = 0; i < m_GeomList.size(); i++)
        m_GeomList[i]->Draw();

}
#endif
