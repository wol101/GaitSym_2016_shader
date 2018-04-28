/*
 *  DataTargetVector.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Tue July 14 2009.
 *  Copyright (c) 1009 Bill Sellers. All rights reserved.
 *
 */

#ifndef DATATARGETVECTOR_H
#define DATATARGETVECTOR_H

#include "DataTarget.h"
#include "PGDMath.h"

class DataTargetVector : public DataTarget
{
public:
    DataTargetVector();
    ~DataTargetVector();

    virtual void SetTargetValues(int size, double *values);
    void SetTargetValues(const char *buf);
    virtual double GetError(int valueListIndex);

    virtual void Dump();

#ifdef USE_QT
    virtual void Draw();
    void SetRadius(double v) { mRadius = v; }
#endif

protected:

    pgd::Vector *m_ValueList;
    int m_ValueListLength;

#ifdef USE_QT
    double mRadius;
    bool mOnlyDrawNext;
#endif

};

#endif // DATATARGETVECTOR_H
