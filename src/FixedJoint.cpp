/*
 *  FixedJoint.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 20/09/2008.
 *  Copyright 2008 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include "FixedJoint.h"
#include "DataFile.h"
#include "Body.h"
#include "Simulation.h"
#include "Util.h"
#include "TIFFWrite.h"

// Simulation global
extern Simulation *gSimulation;


FixedJoint::FixedJoint(dWorldID worldID) : Joint()
{
    m_JointID = dJointCreateFixed(worldID, 0);
    dJointSetData(m_JointID, this);

    dJointSetFeedback(m_JointID, &m_JointFeedback);


}

FixedJoint::~FixedJoint()
{
}

void FixedJoint::SetFixed()
{
    dJointSetFixed (m_JointID);
}


#ifdef USE_QT
void FixedJoint::Draw()
{
}

#endif
