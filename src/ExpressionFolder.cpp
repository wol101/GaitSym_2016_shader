/*
 *  ExpressionFolder.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 06/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#include <stdlib.h>
#include <string.h>
#include <iostream>

#include "ExpressionFolder.h"
#include "ExpressionVar.h"
#include "ExpressionVec.h"
#include "ExpressionMat.h"
#include "ExpressionRef.h"
#include "ExpressionFunTransform3D.h"

#include "DebugControl.h"

ExpressionFolder::ExpressionFolder(std::string type, std::string name, ExpressionFolder *parent)
{
    m_Type = type;
    m_Name = name;
    m_Parent = parent;
    if (parent) m_AlternateTree = parent->GetAlternateTree();
    else m_AlternateTree = 0;
    m_Status = 0;
    m_DefinitionPoint = -1;

    if (gDebug == ParserDebug)
    {
        if (m_Parent)
            std::cerr << "Creating ExpressionFolder " << m_Name << " Parent " << m_Parent->GetPath() << "\n";
        else
            std::cerr << "Creating ExpressionFolder " << m_Name << " Parent " << 0 << "\n";
    }
}

ExpressionFolder::~ExpressionFolder()
{
    std::map<std::string, ExpressionFolder *>::iterator iter;
    for (iter = m_Children.begin(); iter != m_Children.end(); iter++) delete iter->second;
}

ExpressionFolder *ExpressionFolder::AddChild(ExpressionFolder *child)
{
    if (m_Children.find(child->GetName()) != m_Children.end())
    {
        std::cerr << __FILE__ << " " << __LINE__ << " Warning " <<  child->GetName() << " already exists in " << m_Name << "\n";
        return 0;
    }

    m_Children[child->GetName()] = child;
    m_ChildrenList.push_back(child);
    child->SetPathWhenCreated(child->GetPath());
    return child;
}

ExpressionFolder *ExpressionFolder::Search(std::string path, bool useAlternateTree)
{
//#define TRYALTERNATIVEFIRST
#ifdef TRYALTERNATIVEFIRST
    if (m_AlternateTree && useAlternateTree)
    {
        ExpressionFolder *alternativeResult = 0;
        ExpressionFolder *mainResult = 0;
        std::string newPath = GetPathWhenCreated() + "." + path;
        alternativeResult = m_AlternateTree->Search(newPath, false);
        mainResult = Search(path, false);
        if (alternativeResult == 0) return mainResult;
        if (mainResult == 0) return alternativeResult;
        if (alternativeResult->GetDefinitionPoint() > mainResult->GetDefinitionPoint()) return alternativeResult;
        return mainResult;
    }
#endif

    std::map<std::string, ExpressionFolder *>::iterator iter;

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

    int pos = path.find(".");
    if (pos == (int)std::string::npos)
    {
        if (path == "Main")
        {
            if (m_Parent == 0) return this;
            else return m_Parent->Search(path, useAlternateTree);
        }
        iter = m_Children.find(path);
        if (iter == m_Children.end())
        {
#ifndef TRYALTERNATIVEFIRST
            if (m_AlternateTree && useAlternateTree)
            {
                std::string newPath = GetPath() + "." + path;
                ExpressionFolder *r = m_AlternateTree->Search(newPath, useAlternateTree);
                // if (r) std::cerr << newPath << " found in alternate tree\n";
                return r;
            }
#endif
            std::cerr << __FILE__ << " " << __LINE__ << " Warning " << "Could not find " << path << " in " << m_Name << "\n";
            if (gDebug == ParserDebug)
                std::cerr << __FILE__ << " " << __LINE__ << " " << *this << "\n";
            return 0;
        }
        //if (dynamic_cast<ExpressionRef *>(iter->second))
        //{
        //    std::string ref = dynamic_cast<ExpressionRef *>(iter->second)->GetPath();
        //    return Search(ref);
        //}
        return iter->second;
    }

    std::string parent = path.substr(0, pos);
    std::string child = path.substr(pos + 1);
    if (parent == "Main")
    {
        if (m_Parent == 0) return Search(child, useAlternateTree);
        else return m_Parent->Search(path, useAlternateTree);
    }
    iter = m_Children.find(parent);
    if (iter == m_Children.end())
    {
#ifndef TRYALTERNATIVEFIRST
        if (m_AlternateTree && useAlternateTree)
        {
            std::string newPath = GetPath() + "." + path;
            ExpressionFolder *r = m_AlternateTree->Search(newPath, useAlternateTree);
            // if (r) std::cerr << newPath << " found in alternate tree\n";
            return r;
        }
#endif
        std::cerr << __FILE__ << " " << __LINE__ << " Warning " << "Could not find " << path << " in " << m_Name << "\n";
        if (gDebug == ParserDebug)
            std::cerr << __FILE__ << " " << __LINE__ << " " << *this << "\n";
        return 0;
    }
    return iter->second->Search(child, useAlternateTree);
}

std::string ExpressionFolder::GetPath()
{
    if (m_Parent == 0) return m_Name;
    else return m_Parent->GetPath() + "." + m_Name;
}

ExpressionFolder *ExpressionFolder::ParseTokens(std::vector<std::string> &tokens, unsigned int &index)
{
    if (m_Name == "ModelEnvironmentConnection")
        std::cerr << GetPath() << "\n";

    ExpressionFolder *anyFolderPtr;
    index++;
    if (tokens[index] != "=")
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Error Expected = in " << m_Name << "\n";
        return 0;
    }
    index++;
    if (tokens[index] == "{")
    {
        index++;

        while (tokens[index] != "}")
        {
            if (tokens[index + 1] == "&")
            {
                index++;
                ExpressionRef *anyRef = new ExpressionRef("ExpressionRef", tokens[index + 1], this);
                if (anyRef)
                {
                    AddChild(anyRef);
                    anyRef->SetDefinitionPoint(index);
                    if ((anyRef->ParseTokens(tokens, index)) == 0)
                        RemoveChild(anyRef);
                }
            }
            else if (tokens[index] == "ExpressionVar")
            {
                index++;
                ExpressionVar *anyVar = new ExpressionVar(tokens[index - 1], tokens[index], this);
                if (anyVar)
                {
                    AddChild(anyVar);
                    anyVar->SetDefinitionPoint(index);
                    if ((anyVar->ParseTokens(tokens, index)) == 0)
                        RemoveChild(anyVar);
                }
            }
            else if (tokens[index] == "ExpressionVec3")
            {
                index++;
                ExpressionVec *anyVec = new ExpressionVec(tokens[index - 1], tokens[index], this);
                if (anyVec)
                {
                    AddChild(anyVec);
                    anyVec->SetDefinitionPoint(index);
                    if ((anyVec->ParseTokens(tokens, index)) == 0)
                        RemoveChild(anyVec);
                }
            }
            else if (tokens[index] == "ExpressionMat33" || tokens[index] == "ExpressionMatrix")
            {
                index++;
                ExpressionMat *anyMat = new ExpressionMat(tokens[index - 1], tokens[index], this);
                if (anyMat)
                {
                    AddChild(anyMat);
                    anyMat->SetDefinitionPoint(index);
                    if ((anyMat->ParseTokens(tokens, index)) == 0)
                        RemoveChild(anyMat);
                }
            }
            else if (tokens[index] == "ExpressionFunTransform3DIdentity" || tokens[index] == "ExpressionFunTransform3DLin" || tokens[index] == "ExpressionFunTransform3D")
            {
                index++;
                ExpressionFunTransform3D *anyFunTransform3D = new ExpressionFunTransform3D(tokens[index - 1], tokens[index], this);
                if (anyFunTransform3D)
                {
                    AddChild(anyFunTransform3D);
                    anyFunTransform3D->SetDefinitionPoint(index);
                    if ((anyFunTransform3D->ParseTokens(tokens, index)) == 0)
                        RemoveChild(anyFunTransform3D);
                }
            }
            else if (tokens[index] == "ExpressionFolder")
            {
                index++;
                ExpressionFolder *anyFolder = new ExpressionFolder(tokens[index - 1], tokens[index], this);
                if (anyFolder)
                {
                    AddChild(anyFolder);
                    anyFolder->SetDefinitionPoint(index);
                    if ((anyFolder->ParseTokens(tokens, index)) == 0)
                        RemoveChild(anyFolder);
                }
            }

            else if ((anyFolderPtr = Search(tokens[index], false)))
            {
                anyFolderPtr->ParseTokens(tokens, index);
            }
            else
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
            }

            if (index >= tokens.size())
            {
                std::cerr << __FILE__ << " " << __LINE__ << " Error " << "Expected } in " << m_Name << "\n";
                return 0;
            }
        }
        index++;
    }
    else
    {
        while (tokens[index] != ";")
        {
            m_Content += tokens[index];
            index++;
            if (index >= tokens.size())
            {
                std::cerr << __FILE__ << " " << __LINE__ << " Error " << "Expected } in " << m_Name << "\n";
                return 0;
            }
        }
    }

    if (tokens[index] != ";")
    {
        std::cerr << __FILE__ << " " << __LINE__ << " Error " << "Expected ; in " << m_Name << "\n";
        return 0;
    }
    index++;

    return this;
}


void ExpressionFolder::Tokenizer(const char *constbuf, std::vector<std::string> &tokens)
{
    char *ptr = (char *)malloc(strlen(constbuf) + 1);
    strcpy(ptr, constbuf);
    char *qp;
    char byte;
    const char *stopList = "{};,=:&()";
    char oneChar[2] = {0, 0};

    while (*ptr)
    {
        // find non-whitespace
        if (*ptr < 33)
        {
            ptr++;
            continue;
        }

        // is it in stoplist
        if (strchr(stopList, *ptr))
        {
            oneChar[0] = *ptr;
            tokens.push_back(oneChar);
            ptr++;
            continue;
        }

        // is it a double quote?
        if (*ptr == '"')
        {
            ptr++;
            qp = strchr(ptr, '"');
            if (qp)
            {
                *qp = 0;
                tokens.push_back(ptr);
                *qp = '"';
                ptr = qp + 1;
                continue;
            }
        }

        qp = ptr;
        while (*qp >= 33 && strchr(stopList, *qp) == 0 && *qp != '"')
        {
            qp++;
        }
        byte = *qp;
        *qp = 0;
        tokens.push_back(ptr);
        if (byte == 0) break;
        *qp = byte;
        ptr = qp;
    }
}

int ExpressionFolder::RemoveChild(ExpressionFolder *child)
{
    std::map<std::string, ExpressionFolder *>::iterator iter;
    for (iter = m_Children.begin(); iter != m_Children.end(); iter++)
    {
        if (iter->second == child)
        {
            m_Children.erase(iter);
            delete child;
            return 0;
        }
    }
    return __LINE__;
}

void ExpressionFolder::GetAllChildren(std::vector<ExpressionFolder *> *list)
{
    std::map<std::string, ExpressionFolder *>::iterator iter;
    for (iter = m_Children.begin(); iter != m_Children.end(); iter++)
    {
        list->push_back(iter->second);
        iter->second->GetAllChildren(list);
    }
}

void ExpressionFolder::GetAllParents(std::vector<ExpressionFolder *> *list)
{
    if (m_Parent == 0) return;
    list->push_back(m_Parent);
    m_Parent->GetAllParents(list);
}

// output to a stream
std::ostream& operator<<(std::ostream &out, ExpressionFolder &g)
{
    out << g.GetPath() << "\n";
    std::map<std::string, ExpressionFolder *>::iterator iter;
    for (iter = g.m_Children.begin(); iter != g.m_Children.end(); iter++)
    {
        if (dynamic_cast<ExpressionMat *>(iter->second)) out << *dynamic_cast<ExpressionMat *>(iter->second) << "\n";
        else if (dynamic_cast<ExpressionVec *>(iter->second)) out << *dynamic_cast<ExpressionVec *>(iter->second) << "\n";
        else if (dynamic_cast<ExpressionVar *>(iter->second)) out << *dynamic_cast<ExpressionVar *>(iter->second) << "\n";
        else if (dynamic_cast<ExpressionRef *>(iter->second)) out << *dynamic_cast<ExpressionRef *>(iter->second) << "\n";
        else out << *iter->second << "\n";
    }
    return out;
}

// get the value of a contained item. Returns non zero on error
int ExpressionFolder::GetProperty(std::string name, double *value)
{
    ExpressionVar *anyVar = dynamic_cast<ExpressionVar *>(Search(name, false));
    if (anyVar == 0) return __LINE__;
    *value = anyVar->GetValue();
    return 0;
}

int ExpressionFolder::GetProperty(std::string name, pgd::Vector *vector)
{
    ExpressionVec *anyVec = dynamic_cast<ExpressionVec *>(Search(name, false));
    if (anyVec == 0) return __LINE__;
    *vector = anyVec->GetVector();
    return 0;
}

int ExpressionFolder::GetProperty(std::string name, pgd::Matrix3x3 *matrix)
{
    ExpressionMat *anyMat = dynamic_cast<ExpressionMat *>(Search(name, false));
    if (anyMat == 0) return __LINE__;
    *matrix = anyMat->GetMatrix3x3();
    return 0;
}


