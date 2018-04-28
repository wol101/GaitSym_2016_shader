/*
 *  ExpressionFolder.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 06/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#ifndef ExpressionFolder_h
#define ExpressionFolder_h

#include <string>
#include <map>
#include <vector>
#include <iostream>
#include "PGDMath.h"

class ExpressionFolder
{
public:
    ExpressionFolder(std::string type, std::string name, ExpressionFolder *parent);
    virtual ~ExpressionFolder();

    virtual ExpressionFolder *ParseTokens(std::vector<std::string> &tokens, unsigned int &index);

    virtual ExpressionFolder *AddChild(ExpressionFolder *child);
    virtual int RemoveChild(ExpressionFolder *child);
    virtual ExpressionFolder *Search(std::string path, bool useAlternateTree);

    virtual void SetParent(ExpressionFolder *parent) { m_Parent = parent; };
    virtual ExpressionFolder *GetParent() { return m_Parent; };

    virtual std::string GetName() { return m_Name; };
    virtual void SetName(std::string name) { m_Name = name; };
    virtual std::string GetContent() { return m_Content; };
    virtual std::string GetType() { return m_Type; };
    virtual std::string GetPath();
    virtual int GetStatus() { return m_Status; };
    virtual int GetDefinitionPoint() { return m_DefinitionPoint; };
    virtual void SetDefinitionPoint(int definitionPoint) { m_DefinitionPoint = definitionPoint; };
    virtual std::string GetPathWhenCreated() { return m_PathWhenCreated; };
    virtual void SetPathWhenCreated(std::string name) { m_PathWhenCreated = name; };

    virtual int GetProperty(std::string name, double *value);
    virtual int GetProperty(std::string name, pgd::Vector *vector);
    virtual int GetProperty(std::string name, pgd::Matrix3x3 *matrix);

    virtual void GetAllChildren(std::vector<ExpressionFolder *> *list);
    virtual void GetAllParents(std::vector<ExpressionFolder *> *list);

    virtual void SetAlternateTree(ExpressionFolder *anyFolder) { m_AlternateTree = anyFolder; };
    virtual ExpressionFolder *GetAlternateTree() { return m_AlternateTree; };

    virtual pgd::Vector GetGlobalPosition() { return pgd::Vector(0, 0, 0); };
    virtual pgd::Matrix3x3 GetGlobalRotationMatrix() { return pgd::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1); };

    static void Tokenizer(const char *buf, std::vector<std::string> &tokens);

    friend std::ostream& operator<<(std::ostream &out, ExpressionFolder &g);

protected:

    std::map<std::string, ExpressionFolder *> m_Children;
    std::vector<ExpressionFolder *> m_ChildrenList;
    ExpressionFolder *m_Parent;
    std::string m_Name;
    std::string m_Content;
    std::string m_Type;
    std::string m_PathWhenCreated;
    ExpressionFolder *m_AlternateTree;
    int m_Status;
    int m_DefinitionPoint;
};


#endif
