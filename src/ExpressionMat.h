/*
 *  ExpressionMat.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */


#include <vector>
#include "PGDMath.h"
#include "ExpressionFolder.h"

class ExpressionMat: public ExpressionFolder
{
public:
    
    ExpressionMat(std::string type, std::string name, ExpressionFolder *parent);
    
    virtual ExpressionFolder *ParseTokens(std::vector<std::string> &tokens, unsigned int &index);
    
    virtual std::vector<double> *GetData() { return &m_Data; };
    virtual pgd::Matrix3x3 GetMatrix3x3();
    virtual int GetRows() { return m_Rows; };
    virtual int GetCols() { return m_Cols; };
    
    friend std::ostream& operator<<(std::ostream &out, ExpressionMat &g);
    
protected:
        
    std::vector<double> m_Data;
    int m_Rows;
    int m_Cols;
};


