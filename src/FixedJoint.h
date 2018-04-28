/*
 *  FixedJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 20/09/2008.
 *  Copyright 2008 Bill Sellers. All rights reserved.
 *
 */

#ifndef FixedJoint_h
#define FixedJoint_h

#ifdef USE_OPENCL
#include <cl.h>
#endif

#include "Joint.h"
#include "PGDMath.h"


class FixedJoint: public Joint
{
    public:

    FixedJoint(dWorldID worldID);
    ~FixedJoint();

    void SetFixed();


#ifdef USE_QT
    virtual void Draw();
#endif


protected:


};



#endif
