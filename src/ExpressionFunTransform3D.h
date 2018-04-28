/*
 *  ExpressionFunTransform3D.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#ifndef ExpressionFunTransform3D_h
#define ExpressionFunTransform3D_h

#include "ExpressionFolder.h"
#include "PGDMath.h"

class ExpressionFunTransform3D: public ExpressionFolder
{
public:
    
    ExpressionFunTransform3D(std::string type, std::string name, ExpressionFolder *parent);
    
    virtual int Calculate(pgd::Vector *vector);

};

#endif
