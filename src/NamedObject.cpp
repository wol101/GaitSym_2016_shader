/*
 *  NamedObject.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Root object that allows some basic data storage such as naming

#include "NamedObject.h"
#include "FacetedObject.h"
#include "Simulation.h"

#include <ode/ode.h>

#include <sstream>

NamedObject::NamedObject()
{
    m_Visible = true;
    m_Dump = false;
    m_FirstDump = true;
    m_DumpStream = 0;
    m_CaseSensitiveXMLAttributes = false;

#ifdef USE_QT
    m_AxisSize[0] = m_AxisSize[1] = m_AxisSize[2] = 1;
    m_Colour.SetColour(1, 1, 1, 1);
    m_PhysRep = 0;
    m_FirstDraw = true;
#endif
}

NamedObject::~NamedObject()
{
    if (m_DumpStream)
    {
        m_DumpStream->close();
        delete(m_DumpStream);
    }
#ifdef USE_QT
    if (m_PhysRep) delete m_PhysRep;
#endif
}

void NamedObject::Dump()
{
    if (m_Dump == false) return;

    if (m_FirstDump)
    {
        m_FirstDump = false;
        if (m_DumpStream == 0)
        {
            if (m_Name.size() == 0) std::cerr << "NamedObject::Dump error: can only dump a named object\n";
            std::string filename(m_Name);
            filename.append(".dump");
            m_DumpStream = new std::ofstream(filename.c_str());
            m_DumpStream->precision(17);
        }
        if (m_DumpStream)
        {
            *m_DumpStream << "Name\tm_Visible\n";
        }
    }

    if (m_DumpStream)
    {
        *m_DumpStream << m_Name << "\t" << m_Visible << "\n";
    }
}

// this function initialises the data in the object based on the contents
// of a libxml2 node. It uses information from the simulation as required
// to satisfy dependencies
// it returns 0 on success and __LINE__ on failure
int NamedObject::XMLLoad(xmlNodePtr node)
{
    std::string ID;
    GetXMLAttribute(node, "ID", &ID);
    if (ID.length() == 0) goto ERROR_EXIT;
    SetName(ID);
    return 0;

ERROR_EXIT:
    std::ostringstream ss;
    ss << "Error loading XML node: line=" << node->line << " name=" << node->name << " ID=" << ID;
    SetMessage(ss.str());
    return __LINE__;
}

// this function copies the data in the object to a libxml2 node that
// it creates internally. Ownership of this new node is passed to the calling
// routine. It uses information in simulation to control the format used
// and to provide dependencies.
// it returns 0 on success and __LINE__ on failure
xmlNodePtr NamedObject::XMLSave()
{
    xmlNodePtr node = xmlNewNode(0, (const xmlChar *)"OBJECT");
    xmlNewProp(node, (const xmlChar *)"ID", (const xmlChar *)m_Name.c_str());
    return node;
}

// returns the value of a named attribute
// using called provided string
// returns "" if attribute is not found
void NamedObject::GetXMLAttribute(xmlNode *cur, const char *name, std::string *attributeValue)
{
    if (m_CaseSensitiveXMLAttributes)
    {
        for (xmlAttrPtr attr = cur->properties; attr != 0; attr = attr->next)
        {
            if (xmlStrcmp(attr->name, (const xmlChar *)name) == 0)
            {
                *attributeValue = (const char *)attr->children->content;
                return;
            }
        }
    }
    else
    {
        for (xmlAttrPtr attr = cur->properties; attr != 0; attr = attr->next)
        {
            if (xmlStrcasecmp(attr->name, (const xmlChar *)name) == 0)
            {
                *attributeValue = (const char *)attr->children->content;
                return;
            }
        }
    }
    attributeValue->clear();
    return;
}


// returns the value of a named attribute
// using a caller provided char * buffer
// returns a zero length string if attribute is not found
// attributeValueLength is the maximum permitted size of attributeValue including the terminating zero
void NamedObject::GetXMLAttribute(xmlNode *cur, const char *name, char *attributeValue, int attributeValueLength)
{
    if (m_CaseSensitiveXMLAttributes)
    {
        for (xmlAttrPtr attr = cur->properties; attr != 0; attr = attr->next)
        {
            if (xmlStrcmp(attr->name, (const xmlChar *)name) == 0)
            {
                int n = attributeValueLength - 1;
                strncpy(attributeValue, (const char *)attr->children->content, n);
                attributeValue[n] = 0;
                return;
            }
        }
    }
    else
    {
        for (xmlAttrPtr attr = cur->properties; attr != 0; attr = attr->next)
        {
            if (xmlStrcasecmp(attr->name, (const xmlChar *)name) == 0)
            {
                int n = attributeValueLength - 1;
                strncpy(attributeValue, (const char *)attr->children->content, n);
                attributeValue[n] = 0;
                return;
            }
        }
    }
    attributeValue[0] = 0;
}

// replaces the value of a named attribute if it exists
// if it doesn't exist then a new attribute is created
// returns a pointer to the attribute
xmlAttrPtr NamedObject::ReplaceXMLAttribute(xmlNode *cur, const char *name, const char *newValue)
{
    xmlAttrPtr ptr = 0;
    if (m_CaseSensitiveXMLAttributes)
    {
        for (xmlAttrPtr attr = cur->properties; attr != 0; attr = attr->next)
        {
            if (xmlStrcmp(attr->name, (const xmlChar *)name) == 0)
            {
                ptr = attr;
                break;
            }
        }
    }
    else
    {
        for (xmlAttrPtr attr = cur->properties; attr != 0; attr = attr->next)
        {
            if (xmlStrcasecmp(attr->name, (const xmlChar *)name) == 0)
            {
                ptr = attr;
                break;
            }
        }
    }

    if (ptr) xmlRemoveProp(ptr);
    ptr = xmlNewProp(cur, (const xmlChar *)name, (const xmlChar *)newValue);
    return ptr;
}

// removes a named attribute if it exists
void NamedObject::RemoveXMLAttribute(xmlNode *cur, const char *name)
{
    xmlAttrPtr ptr = 0;
    if (m_CaseSensitiveXMLAttributes)
    {
        for (xmlAttrPtr attr = cur->properties; attr != 0; attr = attr->next)
        {
            if (xmlStrcmp(attr->name, (const xmlChar *)name) == 0)
            {
                ptr = attr;
                break;
            }
        }
    }
    else
    {
        for (xmlAttrPtr attr = cur->properties; attr != 0; attr = attr->next)
        {
            if (xmlStrcasecmp(attr->name, (const xmlChar *)name) == 0)
            {
                ptr = attr;
                break;
            }
        }
    }

    if (ptr) xmlRemoveProp(ptr);
}

// returns a pointer to an attribute if it exists
xmlAttrPtr NamedObject::FindXMLAttribute(xmlNode *cur, const char *name)
{
    xmlAttrPtr ptr = 0;
    if (m_CaseSensitiveXMLAttributes)
    {
        for (xmlAttrPtr attr = cur->properties; attr != 0; attr = attr->next)
        {
            if (xmlStrcmp(attr->name, (const xmlChar *)name) == 0)
            {
                ptr = attr;
                break;
            }
        }
    }
    else
    {
        for (xmlAttrPtr attr = cur->properties; attr != 0; attr = attr->next)
        {
            if (xmlStrcasecmp(attr->name, (const xmlChar *)name) == 0)
            {
                ptr = attr;
                break;
            }
        }
    }
    return ptr;
}


