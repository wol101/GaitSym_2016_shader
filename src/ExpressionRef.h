/*
 *  ExpressionRef.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#include "ExpressionFolder.h"
#include "PGDMath.h"

class ExpressionRef: public ExpressionFolder
{
public:
    
    ExpressionRef(std::string type, std::string name, ExpressionFolder *parent);
    
    virtual ExpressionFolder *ParseTokens(std::vector<std::string> &tokens, unsigned int &index);
    virtual ExpressionFolder *Search(std::string path, bool useAlternateTree);
    
    virtual std::string GetLink() { return m_Link; };
    virtual ExpressionFolder *GetLinked(bool useAlternateTree);
    
protected:
        
    std::string m_Link;
};


