/*
 *  ExpressionRef.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#include <iostream>
#include "ExpressionRef.h"
#include "ExpressionVar.h"
#include "ExpressionVec.h"
#include "ExpressionMat.h"
#include "ExpressionFunTransform3D.h"

#include "DebugControl.h"

ExpressionRef::ExpressionRef(std::string type, std::string name, ExpressionFolder *parent): ExpressionFolder(type, name, parent)
{
}

ExpressionFolder *ExpressionRef::ParseTokens(std::vector<std::string> &tokens, unsigned int &index)
{
    if (tokens[index] == "&") // this is the definition
    {
        index++;
        index++;
        if (tokens[index] != "=") 
        {
            std::cerr << __FILE__ << " " << __LINE__ << " Error " << "Expected = in " << m_Name << "\n";
            return 0;
        }
        index++;
        
        m_Link = "";
        while (tokens[index] != ";")
        {
            m_Link += tokens[index];
            index++;
        }
        index++;
        
        if (gDebug == ParserDebug)
        {
            std::cerr << "Created ExpressionRef " << m_Name << "\n";
            std::cerr << m_Type << " " << m_Link << "\n";
        }
        return this;
    }
    
    // but here it is being used
    std::cerr << GetPath() << "\n";
    ExpressionFolder *anyFolder = GetLinked(false);
    if (anyFolder == 0) 
    {
        std::cerr << __FILE__ << " " << __LINE__ << " Warning " << tokens[index] << " not found in " << m_Name << "\n";
        index++;
        int braceCount = 0;
        std::string content;
        while (tokens[index] != ";" || braceCount != 0)
        {
            if (tokens[index] == "{") braceCount++;
            else if (tokens[index] == "}") braceCount--;
            content += tokens[index];
            index++;
        }
        std::cerr << __FILE__ << " " << __LINE__ << " Warning ignoring" << "\n";
        std::cerr << content << "\n";
        index++;
        return 0;
    }
    if (gDebug == ParserDebug)
    {
        std::cerr << "Reference redirected from " << GetPath() << " to " << anyFolder->GetPath() << "\n";
    }
    // now need to temporarily change parent
    ExpressionFolder *parentStore = anyFolder->GetParent();
    std::string nameStore = anyFolder->GetName();
    anyFolder->SetParent(m_Parent);
    anyFolder->SetName(m_Name);
    ExpressionFolder *retValue = anyFolder->ParseTokens(tokens, index);
    anyFolder->SetParent(parentStore);
    anyFolder->SetName(nameStore);
    return retValue;
}

ExpressionFolder *ExpressionRef::Search(std::string path, bool useAlternateTree)
{
    // if searching up, just search up
    if (path[0] == '.')
    {
        if (m_Parent == 0)
        {
            std::cerr << __FILE__ << " " << __LINE__ << " Warning " << "Reached root folder without finding " << path << "\n";
            return 0;
        }
        std::string newPath = path.substr(1);
        return m_Parent->Search(newPath, useAlternateTree);
    }
    
    // but if searching down add in the link
    std::string newPath = m_Link + "." + path;
    if (gDebug == ParserDebug)
    {
        std::cerr << "Reference converted " << path << " to " << newPath << "\n";
    }
    return m_Parent->Search(newPath, useAlternateTree);
}

ExpressionFolder *ExpressionRef::GetLinked(bool useAlternateTree)
{
    ExpressionFolder *anyFolder = m_Parent->Search(m_Link, useAlternateTree);
    if (anyFolder == 0) return 0;
    
    ExpressionRef *anyRef = dynamic_cast<ExpressionRef *>(anyFolder);
    if (anyRef) return anyRef->GetLinked(useAlternateTree);
    
    return anyFolder;
}

// output to a stream
std::ostream& operator<<(std::ostream &out, ExpressionRef &g)
{
    out << "ExpressionRef " << g.GetPath() << " " << g.GetLink();
    return out;
}
