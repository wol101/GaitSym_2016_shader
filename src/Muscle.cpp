/*
 *  Muscle.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>

#ifdef USE_QT
#include "GLUtils.h"
extern int gActivationDisplay;
extern int gElasticDisplay;
extern std::string gOBJName;
#endif

#include "Muscle.h"

Muscle::Muscle(Strap *strap)
{
    m_Strap = strap;
#ifdef USE_QT
    m_ElasticEnergyColourFullScale = 50.0;
#endif
}


Muscle::~Muscle()
{
    delete m_Strap;
}

#ifdef USE_QT
void Muscle::Draw()
{
    gOBJName = m_Name;
    if (gActivationDisplay)
    {
        if (gElasticDisplay == false)
        {
            Colour colour;
            float muscleAlpha = GetColour()->alpha;
            float muscleForceAlpha = GetForceColour()->alpha;
            GLUtils::SetColourFromMap(GetActivation(), JetColourMap, &colour, true);
            colour.alpha = muscleAlpha;
            m_Strap->SetColour(colour);
            colour.alpha = muscleForceAlpha;
            m_Strap->SetForceColour(colour);
        }
        else
        {
            Colour colour;
            float muscleAlpha = GetColour()->alpha;
            float muscleForceAlpha = GetForceColour()->alpha;
            GLUtils::SetColourFromMap(GetElasticEnergy() / m_ElasticEnergyColourFullScale, JetColourMap, &colour);
            colour.alpha = muscleAlpha;
            m_Strap->SetColour(colour);
            colour.alpha = muscleForceAlpha;
            m_Strap->SetForceColour(colour);
        }
    }
    else
    {
        m_Strap->SetColour(m_Colour);
        m_Strap->SetForceColour(m_ForceColour);
    }
    m_Strap->Draw();
}
#endif
