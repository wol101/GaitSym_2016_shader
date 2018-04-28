/*
 *  Contact.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/02/2002.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef Contact_h
#define Contact_h

#include "NamedObject.h"

class Contact:public NamedObject
{
public:

    void SetJointID(dJointID jointID) { m_JointID = jointID; };

    dJointID GetJointID() { return m_JointID; };
    dJointFeedback* GetJointFeedback() { return &m_ContactJointFeedback; };
    dVector3* GetContactPosition() { return &m_ContactPosition; };

#ifdef USE_QT
    void Draw();
    void SetForceRadius(float forceSize) { m_ForceRadius = forceSize; };
    void SetForceScale(float forceScale) { m_ForceScale = forceScale; };
#endif

protected:

    dJointID m_JointID;
    dJointFeedback m_ContactJointFeedback;
    dVector3 m_ContactPosition;

#ifdef USE_QT
    float m_ForceRadius;
    float m_ForceScale;
#endif
};


#endif

