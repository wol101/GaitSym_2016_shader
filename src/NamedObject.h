/*
 *  NamedObject.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Root object that allows some basic data storage such as naming

#ifndef NamedObject_h
#define NamedObject_h

#include <string>
#include <iostream>
#include <sstream>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#ifdef USE_QT
#include "GLUtils.h"
#endif

class FacetedObject;
class Simulation;

class NamedObject
{
public:

    NamedObject();
    virtual ~NamedObject();

    void SetName(const char* name) { m_Name = name; }
    void SetName(const unsigned char* name) { m_Name = (const char *)name; }
    void SetName(const std::string &name) { m_Name = name; }
    void SetName(const std::string *name) { m_Name = *name; }
    const std::string *GetName() { return &m_Name; }

    bool GetVisible() { return m_Visible; }
    void SetVisible(bool v) { m_Visible = v; }

    bool GetDump() { return m_Dump; }
    void SetDump(bool v) { m_Dump = v; }

    bool GetCaseSensitiveXMLAttributes() { return m_CaseSensitiveXMLAttributes; }
    void SetCaseSensitiveXMLAttributes(bool v) { m_CaseSensitiveXMLAttributes = v; }

    void SetMessage(const char* message) { m_Message = message; }
    void SetMessage(const unsigned char* message) { m_Message = (const char *)message; }
    void SetMessage(const std::string &message) { m_Message = message; }
    void SetMessage(const std::string *message) { m_Message = *message; }
    const std::string *GetMessage() { return &m_Message; }

    virtual void Dump();
    virtual int XMLLoad(xmlNodePtr node);
    virtual xmlNodePtr XMLSave();

#ifdef USE_QT
    void SetAxisSize(float axisSize[3]) {m_AxisSize[0] = axisSize[0]; m_AxisSize[1] = axisSize[1]; m_AxisSize[2] = axisSize[2]; }
    void SetColour(Colour &colour) { m_Colour = colour; }
    void SetColour(double r, double g, double b, double alpha) { m_Colour.SetColour(r, g, b, alpha); }
    const float *GetAxisSize() { return m_AxisSize; }
    const Colour *GetColour() { return &m_Colour; }
    FacetedObject *GetPhysRep() { return m_PhysRep; }
#endif

protected:

    void GetXMLAttribute(xmlNode *cur, const char *name, std::string *attributeValue);
    void GetXMLAttribute(xmlNode *cur, const char *name, char *attributeValue, int attributeValueLength);
    xmlAttrPtr ReplaceXMLAttribute(xmlNode *cur, const char *name, const char *newValue);
    void RemoveXMLAttribute(xmlNode *cur, const char *name);
    xmlAttrPtr FindXMLAttribute(xmlNode *cur, const char *name);

    std::string m_Name;
    std::string m_Message;

    bool m_Visible;

    bool m_Dump;
    bool m_FirstDump;
    std::ofstream *m_DumpStream;

    bool m_CaseSensitiveXMLAttributes;

#ifdef USE_QT
    float m_AxisSize[3];
    Colour m_Colour;
    FacetedObject *m_PhysRep;
    bool m_FirstDraw;
#endif
};

#endif
