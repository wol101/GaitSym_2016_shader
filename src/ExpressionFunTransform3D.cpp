/*
 *  ExpressionFunTransform3D.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#include <iostream>
#include "ExpressionFunTransform3D.h"
#include "ExpressionMat.h"
#include "ExpressionVec.h"

#include "DebugControl.h"

ExpressionFunTransform3D::ExpressionFunTransform3D(std::string type, std::string name, ExpressionFolder *parent): ExpressionFolder(type, name, parent)
{
    unsigned int index = 0;
    ExpressionFolder *anyFolder;
    std::vector<std::string> tokens;
    
    anyFolder = new ExpressionMat("ExpressionMat", "ScaleMat", this);
    Tokenizer("ScaleMat={{1,0,0},{0,1,0},{0,0,1}};", tokens);
    anyFolder->ParseTokens(tokens, index);
    AddChild(anyFolder);
    
    tokens.clear();
    index = 0;
    anyFolder = new ExpressionVec("ExpressionVec", "Offset", this);
    Tokenizer("Offset={0,0,0};", tokens);
    anyFolder->ParseTokens(tokens, index);
    AddChild(anyFolder);
    
    
}

int ExpressionFunTransform3D::Calculate(pgd::Vector *vector)
{
    ExpressionVec *offset = dynamic_cast<ExpressionVec *>(m_Children["Offset"]);
    if (offset == 0) return __LINE__;
    if (offset->GetStatus() != 0) return __LINE__;
    ExpressionMat *scalemat = dynamic_cast<ExpressionMat *>(m_Children["ScaleMat"]);
    if (scalemat == 0) return __LINE__;
    if (scalemat->GetStatus() != 0) return __LINE__;

    pgd::Vector offsetVector = *vector - offset->GetVector();
    pgd::Vector scaledVector = scalemat->GetMatrix3x3() * offsetVector;
    *vector = scaledVector + offset->GetVector();
    return 0;
}

