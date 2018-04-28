/*
 *  Contact.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/02/2006.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>
#include <vector>

#include "Contact.h"
#include "PGDMath.h"
#include "FacetedPolyline.h"

#ifdef USE_QT
#include "GLUtils.h"
extern int gDrawContactForces;
extern std::string gOBJName;
#endif

// length of vector a
#define LENGTHOF(a) \
        sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

#define LENGTH2OF(a) \
        (a[0]*a[0]+a[1]*a[1]+a[2]*a[2])


#ifdef USE_QT
void Contact::Draw()
{
    if (m_Visible == false) return;

    const int kSides = 128;

    if (gDrawContactForces)
    {
        gOBJName = m_Name;
        GLUtils::DrawAxes(m_AxisSize[0], m_AxisSize[1], m_AxisSize[2],
                m_ContactPosition[0], m_ContactPosition[1], m_ContactPosition[2]);

        std::vector<pgd::Vector> polyline;
        pgd::Vector startPos(m_ContactPosition[0], m_ContactPosition[1], m_ContactPosition[2]);
        pgd::Vector length(m_ContactJointFeedback.f1[0] * m_ForceScale, m_ContactJointFeedback.f1[1] * m_ForceScale, m_ContactJointFeedback.f1[2] * m_ForceScale);
        polyline.push_back(startPos);
        polyline.push_back(startPos + length);
        m_PhysRep = new FacetedPolyline(&polyline, m_ForceRadius, kSides);
        m_PhysRep->SetColour(m_Colour);
        m_PhysRep->Draw();
    }
}
#endif
