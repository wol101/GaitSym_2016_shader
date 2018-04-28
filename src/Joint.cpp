/*
 *  Joint.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <assert.h>
#include <ode/ode.h>

#include "Joint.h"
#include "Body.h"

Joint::Joint()
{
    m_Body1 = 0;
    m_Body2 = 0;
}

Joint::~Joint()
{
    dJointDestroy (m_JointID);
}

void Joint::Attach(Body *body1, Body *body2)
{
    assert(body1 != 0 || body2 != 0);
    m_Body1 = body1;
    m_Body2 = body2;
    if (m_Body1 == 0) dJointAttach(m_JointID, 0, m_Body2->GetBodyID());
    else if (m_Body2 == 0) dJointAttach(m_JointID, m_Body1->GetBodyID(), 0);
    else dJointAttach(m_JointID, m_Body1->GetBodyID(), m_Body2->GetBodyID());
}

dJointFeedback *Joint::GetFeedback()
{
    return dJointGetFeedback(m_JointID);
}


