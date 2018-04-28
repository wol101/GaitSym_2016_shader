/*
 *  ExpressionVec.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 08/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#include <iostream>
#include "ExpressionVec.h"
#include "ExpressionParser.h"

#include "DebugControl.h"

ExpressionVec::ExpressionVec(std::string type, std::string name, ExpressionFolder *parent): ExpressionFolder(type, name, parent)
{
}

pgd::Vector ExpressionVec::GetVector()
{
    pgd::Vector v(0, 0, 0);
    
    if (m_Data.size() > 0) v.x = m_Data[0];
    if (m_Data.size() > 1) v.y = m_Data[1];
    if (m_Data.size() > 2) v.z = m_Data[2];
    
    return v;
}

ExpressionFolder *ExpressionVec::ParseTokens(std::vector<std::string> &tokens, unsigned int &index)
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
            break;*/
            
        case EPVectorVal:
            m_Data.push_back(v.vector.x);
            m_Data.push_back(v.vector.y);
            m_Data.push_back(v.vector.z);
            break;
            
/*        case EPMatrix3x3Val:
            m_Data.push_back(v.matrix.e11);
            m_Data.push_back(v.matrix.e12);
            m_Data.push_back(v.matrix.e13);
            m_Data.push_back(v.matrix.e21);
            m_Data.push_back(v.matrix.e22);
            m_Data.push_back(v.matrix.e23);
            m_Data.push_back(v.matrix.e31);
            m_Data.push_back(v.matrix.e32);
            m_Data.push_back(v.matrix.e33);
            break;*/
            
            
        default:
            //expressionParser.SubstituteNames(&expression);
            std::cerr << __FILE__ << " " << __LINE__ << " Warning " << "Incorrect return type for " << expression << " in " << m_Name << "\n";
            m_Content = expression;
            m_Status = 1;
            return this;
    }
    
    if (gDebug == ParserDebug)
    {
        std::cerr << "Created ExpressionVec " << GetPath() << "\n";
        for (unsigned int i = 0; i < m_Data.size(); i++) std::cerr << m_Data[i] << "\n";
    }
    
    return this;
}

// output to a stream
std::ostream& operator<<(std::ostream &out, ExpressionVec &g)
{
    out << "ExpressionVec " << g.GetPath() << " ";
    pgd::Vector v = g.GetVector();
    out << "[" << v.x << ", " << v.y << ", " << v.z << "]";
    return out;
}
