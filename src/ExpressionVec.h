/*
 *  ExpressionVec.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 08/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#include <vector>
#include "PGDMath.h"
#include "ExpressionFolder.h"

class ExpressionVec: public ExpressionFolder
{
public:
    
    ExpressionVec(std::string type, std::string name, ExpressionFolder *parent);
    
    virtual ExpressionFolder *ParseTokens(std::vector<std::string> &tokens, unsigned int &index);

    virtual std::vector<double> *GetData() { return &m_Data; };
    virtual pgd::Vector GetVector();
    
    friend std::ostream& operator<<(std::ostream &out, ExpressionVec &g);
    
protected:
    
    std::vector<double> m_Data;
    
};

