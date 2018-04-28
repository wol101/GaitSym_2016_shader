/*
 *  BoxGeom.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 30/05/2012.
 *  Copyright 2012 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>

#include "BoxGeom.h"

#ifdef USE_QT
#include "FacetedBox.h"
extern int gDrawGeomsFlag;
extern std::string gOBJName;
#endif

BoxGeom::BoxGeom(dSpaceID space, double lx, double ly, double lz)
{
    // create the geom
    m_GeomID = dCreateBox(space, lx, ly, lz);
    dGeomSetData(m_GeomID, this);
}

#ifdef USE_QT
void BoxGeom::Draw()
{
    if (m_Visible == false) return;

    // perform late initialisation
    if (m_FirstDraw)
    {
        m_FirstDraw = false;
        // get the dimensions
        dVector3 result;
        dGeomBoxGetLengths (m_GeomID, result);

        m_PhysRep = new FacetedBox(result[0], result[1], result[2]);
        m_PhysRep->SetColour(m_Colour);

    }

    if (gDrawGeomsFlag)
    {
        gOBJName = m_Name;

        const double *bodyRotation = dBodyGetRotation(dGeomGetBody(m_GeomID));
        const double *boxRelPosition = dGeomGetOffsetPosition(m_GeomID);
        const double *boxRelRotation = dGeomGetOffsetRotation(m_GeomID);

        dVector3 p;
        dMatrix3 r;

        // get the box position in world coordinates
        dBodyGetRelPointPos(dGeomGetBody(m_GeomID), boxRelPosition[0], boxRelPosition[1], boxRelPosition[2], p);

        //combine the body rotation with the box rotation to get combined rotation from world coordinates
        dMultiply0(r, bodyRotation, boxRelRotation, 3, 3, 3);

        // and draw the box
        m_PhysRep->SetColour(m_Colour);
        m_PhysRep->SetDisplayRotation(r);
        m_PhysRep->SetDisplayPosition(p[0], p[1], p[2]);
        m_PhysRep->Draw();
    }
}
#endif
