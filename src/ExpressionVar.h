/*
 *  ExpressionVar.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */


#include "PGDMath.h"
#include "ExpressionFolder.h"

class ExpressionVar: public ExpressionFolder
{
public:
    
    ExpressionVar(std::string type, std::string name, ExpressionFolder *parent);
    
    virtual ExpressionFolder *ParseTokens(std::vector<std::string> &tokens, unsigned int &index);
    
    virtual double GetValue() { return m_Value; };

    friend std::ostream& operator<<(std::ostream &out, ExpressionVar &g);

protected:
        
        double m_Value;
    
};

