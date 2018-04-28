/*
 *  ExpressionMat.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#include <iostream>
#include "ExpressionMat.h"
#include "ExpressionParser.h"

#include "DebugControl.h"

ExpressionMat::ExpressionMat(std::string type, std::string name, ExpressionFolder *parent): ExpressionFolder(type, name, parent)
{
    m_Rows = 0;
    m_Cols = 0;
}

pgd::Matrix3x3 ExpressionMat::GetMatrix3x3()
{
    pgd::Matrix3x3 v(0, 0, 0, 0, 0, 0, 0, 0, 0);
    
    if (m_Data.size() > 0) v.e11 = m_Data[0];
    if (m_Data.size() > 1) v.e12 = m_Data[1];
    if (m_Data.size() > 2) v.e13 = m_Data[2];
    if (m_Data.size() > 3) v.e21 = m_Data[3];
    if (m_Data.size() > 4) v.e22 = m_Data[4];
    if (m_Data.size() > 5) v.e23 = m_Data[5];
    if (m_Data.size() > 6) v.e31 = m_Data[6];
    if (m_Data.size() > 7) v.e32 = m_Data[7];
    if (m_Data.size() > 8) v.e33 = m_Data[8];
    
    return v;
}

ExpressionFolder *ExpressionMat::ParseTokens(std::vector<std::string> &tokens, unsigned int &index)
{
    m_Status = 0;
    
    index++;
    if (tokens[index] != "=") 
    {
        std::cerr << __FILE__ << " " << __LINE__ << " Error " << "Expected = in " << m_Name << "\n";
        return 0;
    }
    index++;
    
    std::string expression;
    while (tokens[index] != ";")
    {
        expression += tokens[index];
        index++;
    }
    index++;
    ExpressionParser expressionParser;
    expressionParser.SetFolder(m_Parent);
    if (expressionParser.CreateFromString(expression))
    {
        //expressionParser.SubstituteNames(&expression);
        std::cerr << __FILE__ << " " << __LINE__ << " Warning " << "Error parsing " << expression << " in " << m_Name << "\n";
        m_Content = expression;
        m_Status = 1;
        return this;
    }
    ExpressionParserValue v = expressionParser.Evaluate();
    m_Data.clear();
    switch (v.type)
    {
/*        case EPNumber:
            m_Data.push_back(v.value);
            m_Rows = 1;
            m_Cols = 1;
            break;
            
        case EPVectorVal:
            m_Data.push_back(v.vector.x);
            m_Data.push_back(v.vector.y);
            m_Data.push_back(v.vector.z);
            m_Rows = 3;
            m_Cols = 1;
            break;*/
            
        case EPMatrix3x3Val:
            m_Data.push_back(v.matrix.e11);
            m_Data.push_back(v.matrix.e12);
            m_Data.push_back(v.matrix.e13);
            m_Data.push_back(v.matrix.e21);
            m_Data.push_back(v.matrix.e22);
            m_Data.push_back(v.matrix.e23);
            m_Data.push_back(v.matrix.e31);
            m_Data.push_back(v.matrix.e32);
            m_Data.push_back(v.matrix.e33);
            m_Rows = 3;
            m_Cols = 3;
            break;
            
        default:
            //expressionParser.SubstituteNames(&expression);
            std::cerr << __FILE__ << " " << __LINE__ << " Warning " << "Incorrect return type for " << expression << " in " << m_Name << "\n";
            m_Content = expression;
            m_Status = 1;
            return this;
    }
    
    if (gDebug == ParserDebug)
    {
        std::cerr << "Created ExpressionMat " << GetPath() << "\n";
        for (unsigned int i = 0; i < m_Data.size(); i++) std::cerr << m_Data[i] << "\n";
    }
    return this;
}

// output to a stream
std::ostream& operator<<(std::ostream &out, ExpressionMat &g)
{
    out << "ExpressionMat " << g.GetPath() << " ";
    pgd::Matrix3x3 m = g.GetMatrix3x3();
    // out << "[[" << m.e11 << ", " << m.e12 << ", " << m.e13 << "],\n [" << m.e21 << ", " << m.e22 << ", " << m.e23 << "],\n [" << m.e31 << ", " << m.e32 << ", " << m.e33 << "]]" << "\n";
    out << "[[" << m.e11 << ", " << m.e12 << ", " << m.e13 << "], [" << m.e21 << ", " << m.e22 << ", " << m.e23 << "], [" << m.e31 << ", " << m.e32 << ", " << m.e33 << "]]";
    return out;
}
