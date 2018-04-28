/*
 *  XMLConverter.cc
 *  GA
 *
 *  Created by Bill Sellers on Fri Dec 12 2003.
 *  Copyright (c) 2003 Bill Sellers. All rights reserved.
 *
 */

#include <stdlib.h>
#include <string.h>
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <sstream>
#include <cfloat>
#include <cmath>
#include <assert.h>
#include <iostream>

#include "XMLConverter.h"
#include "DataFile.h"
#include "ExpressionParser.h"

XMLConverter::XMLConverter()
{
    m_Doc = 0;
    m_DocTxtPtr = 0;
    m_DTDValidateFlag = false;
    m_ConversionType = SmartSubstitution;
    m_SmartSubstitutionTextBuffer = 0;
    m_SmartSubstitutionFlag = false;
}

XMLConverter::XMLConverter(XMLConverter &converter)
{
    if (converter.m_Doc) m_Doc = xmlCopyDoc(converter.m_Doc, 1);
    else m_Doc = 0;

    // need to reapply genome to copy
    m_DocTxtPtr = 0;
    m_DocTxtLen = 0;

    m_DTDValidateFlag = converter.m_DTDValidateFlag;
    m_ConversionType = converter.m_ConversionType;

    unsigned int i;
    unsigned int len = 0;
    std::string *str;
    ExpressionParser *parser;
    for (i = 0; i < converter.m_SmartSubstitutionTextComponents.size(); i++)
    {
        str = new std::string(*converter.m_SmartSubstitutionTextComponents[i]);
        m_SmartSubstitutionTextComponents.push_back(str);
        len += str->size();
    }
    for (i = 0; i < converter.m_SmartSubstitutionParserComponents.size(); i++)
    {
        parser = new ExpressionParser(*converter.m_SmartSubstitutionParserComponents[i]);
        m_SmartSubstitutionParserComponents.push_back(parser);
    }
    m_SmartSubstitutionValues = converter.m_SmartSubstitutionValues;

    if (converter.m_SmartSubstitutionTextBuffer) m_SmartSubstitutionTextBuffer = new char[len + m_SmartSubstitutionValues.size() * 64];
    else m_SmartSubstitutionTextBuffer = 0;

}

XMLConverter::~XMLConverter()
{
    unsigned int i;
    if (m_Doc) xmlFreeDoc(m_Doc);
    if (m_DocTxtPtr) xmlFree(m_DocTxtPtr);
    if (m_SmartSubstitutionTextBuffer) delete m_SmartSubstitutionTextBuffer;
    for (i = 0; i < m_SmartSubstitutionTextComponents.size(); i++)
        delete m_SmartSubstitutionTextComponents[i];
    for (i = 0; i < m_SmartSubstitutionParserComponents.size(); i++)
        delete m_SmartSubstitutionParserComponents[i];
}

// load the base file for smart substitution file
int XMLConverter::LoadBaseXMLFile(const char *filename)
{
    unsigned int i;
    if (m_Doc) xmlFreeDoc(m_Doc);
    if (m_DocTxtPtr) xmlFree(m_DocTxtPtr);
    if (m_SmartSubstitutionTextBuffer) delete m_SmartSubstitutionTextBuffer;
    for (i = 0; i < m_SmartSubstitutionTextComponents.size(); i++)
        delete m_SmartSubstitutionTextComponents[i];
    for (i = 0; i < m_SmartSubstitutionParserComponents.size(); i++)
        delete m_SmartSubstitutionParserComponents[i];
    m_SmartSubstitutionTextComponents.clear();
    m_SmartSubstitutionParserComponents.clear();
    m_SmartSubstitutionValues.clear();

    if (m_ConversionType == SmartSubstitution)
    {
        DataFile smartSubstitutionBaseXMLFile;
        if (smartSubstitutionBaseXMLFile.ReadFile(filename)) return 1;
        DoSmartSubstitution(smartSubstitutionBaseXMLFile.GetRawData());
    }

    return 0;
}

// load the base XML for smart substitution file
int XMLConverter::LoadBaseXMLString(char *string)
{
    unsigned int i;
    if (m_Doc) xmlFreeDoc(m_Doc);
    if (m_DocTxtPtr) xmlFree(m_DocTxtPtr);
    if (m_SmartSubstitutionTextBuffer) delete m_SmartSubstitutionTextBuffer;
    for (i = 0; i < m_SmartSubstitutionTextComponents.size(); i++)
        delete m_SmartSubstitutionTextComponents[i];
    for (i = 0; i < m_SmartSubstitutionParserComponents.size(); i++)
        delete m_SmartSubstitutionParserComponents[i];
    m_SmartSubstitutionTextComponents.clear();
    m_SmartSubstitutionParserComponents.clear();
    m_SmartSubstitutionValues.clear();

    if (m_ConversionType == SmartSubstitution)
    {
        DoSmartSubstitution(string);
    }

    return 0;
}

xmlChar *XMLConverter::GetFormattedXML(int *docTxtLen)
{
    if (m_ConversionType == SmartSubstitution)
    {
        if (m_SmartSubstitutionTextBuffer == 0) return 0;
        char *ptr = m_SmartSubstitutionTextBuffer;
        unsigned int i;
        int chars;
        for (i = 0; i < m_SmartSubstitutionValues.size(); i++)
        {
            chars = sprintf(ptr, "%s", m_SmartSubstitutionTextComponents[i]->c_str());
            ptr += chars;
            chars = sprintf(ptr, "%.17e", m_SmartSubstitutionValues[i]);
            ptr += chars;
        }
        chars = sprintf(ptr, "%s", m_SmartSubstitutionTextComponents[i]->c_str());
        ptr += chars;
        *docTxtLen = (int)(ptr - m_SmartSubstitutionTextBuffer);
        return (xmlChar *)m_SmartSubstitutionTextBuffer;
    }

    return 0;
}

// this needs to be customised depending on how the genome interacts with
// the XML file specifying the simulation
int XMLConverter::ApplyGenome(int genomeSize, double *genomeData)
{
    if (m_ConversionType == SmartSubstitution)
    {
        ExpressionParser *parser;
        for (unsigned int i = 0; i < m_SmartSubstitutionParserComponents.size(); i++)
        {
            // set up the genome as a function v(locus) and evaluate
            parser = new ExpressionParser(*m_SmartSubstitutionParserComponents[i]);
            parser->SetVData(genomeSize, genomeData);
            ExpressionParserValue value = parser->Evaluate();
            if (value.type == EPNumber)
            {
                m_SmartSubstitutionValues[i] = value.value;
            }
            else
            {
                std::cerr << "Error: XMLConverter::ApplyGenome m_SmartSubstitutionParserComponents[" << i << "] does not evaluate to number\n";
                std::cerr << "Applying standard fix up and setting to zero\n";
                m_SmartSubstitutionValues[i] = 0;
            }
            delete parser;
        }
    }

    return 0;
}

void XMLConverter::NewCyclicDriver(xmlNodePtr parent, char *target,
                     int n, double *d, double *v, double phase)
{
    xmlNodePtr newNode;
    xmlAttrPtr newAttr;
    char buffer[256];
    std::ostringstream out;
    int i;
    out.precision(17);
    out.setf( std::ios::scientific );
    assert(phase >= 0 && phase <= 1);

    newNode = xmlNewTextChild(parent, 0, (xmlChar *)"CYCLIC_DRIVER", 0);
    strncpy(buffer, target, 200);
    strcat(buffer, "Driver");
    newAttr = xmlNewProp(newNode, (xmlChar *)"Name", (xmlChar *)buffer);
    newAttr = xmlNewProp(newNode, (xmlChar *)"Target", (xmlChar *)target);

    if (phase != 0)
    {
        double *d2 = new double[n + 1];
        double *v2 = new double[n + 1];
        double totalDuration = 0;
        for (i = 0; i < n; i++) totalDuration += d[i];
        double durationOffset = totalDuration * phase;
        totalDuration = 0;
        for (i = 0; i < n; i++)
        {
            totalDuration += d[i];
            if (totalDuration >= durationOffset) break;
        }
        int split = i;
        int index = 0;
        d2[index] = totalDuration - durationOffset;
        v2[index] = v[split];
        index++;
        for (i = split + 1; i < n; i++)
        {
            d2[index] = d[i];
            v2[index] = v[i];
            index++;
        }
        for (i = 0; i < split; i++)
        {
            d2[index] = d[i];
            v2[index] = v[i];
            index++;
        }
        d2[index] = d[split] - d2[0];
        v2[index] = v[split];

        for (i = 0; i < n + 1; i++) out << d2[i] << " " << v2[i] << " ";

        delete [] d2;
        delete [] v2;
    }
    else
    {
        for (i = 0; i < n; i++) out << d[i] << " " << v[i] << " ";
    }

    newAttr = xmlNewProp(newNode, (xmlChar *)"DurationValuePairs", (xmlChar *)out.str().c_str());
}

void XMLConverter::NewStepDriver(xmlNodePtr parent, char *target,
                                   int n, double *d, double *v)
{
    xmlNodePtr newNode;
    xmlAttrPtr newAttr;
    char buffer[256];
    std::ostringstream out;
    int i;
    out.precision(17);
    out.setf( std::ios::scientific );

    newNode = xmlNewTextChild(parent, 0, (xmlChar *)"STEP_DRIVER", 0);
    strncpy(buffer, target, 200);
    strcat(buffer, "Driver");
    newAttr = xmlNewProp(newNode, (xmlChar *)"Name", (xmlChar *)buffer);
    newAttr = xmlNewProp(newNode, (xmlChar *)"Target", (xmlChar *)target);

    for (i = 0; i < n; i++) out << d[i] << " " << v[i] << " ";

    newAttr = xmlNewProp(newNode, (xmlChar *)"DurationValuePairs", (xmlChar *)out.str().c_str());
}

void XMLConverter::FlexorEncode(double *in, int n, double *out)
{
    int i;
    for (i = 0; i < n; i++) out[i] = in[i] > 0 ? in[i]: 0;
}

void XMLConverter::ExtensorEncode(double *in, int n, double *out)
{
    int i;
    for (i = 0; i < n; i++) out[i] = in[i] < 0 ? -in[i]: 0;
}

// perform the smart substitution
// returns 0 on no error
void XMLConverter::DoSmartSubstitution(char *dataPtr)
{
    char *ptr1 = dataPtr;
    std::string *s;
    ExpressionParser *expressionParser;
    int length = strlen(dataPtr);

    char *ptr2 = strstr(ptr1, "[[");
    while (ptr2)
    {
        *ptr2 = 0;
        s = new std::string(ptr1);
        m_SmartSubstitutionTextComponents.push_back(s);

        ptr2 += 2;
        ptr1 = strstr(ptr2, "]]");
        if (ptr2 == 0)
        {
            std::cerr << "Error: could not find matching ]]\n";
            exit(1);
        }
        expressionParser = new ExpressionParser();
        expressionParser->CreateFromString(ptr2, ptr1 - ptr2);
        m_SmartSubstitutionParserComponents.push_back(expressionParser);
        m_SmartSubstitutionValues.push_back(0); // dummy values
        ptr1 += 2;
        ptr2 = strstr(ptr1, "[[");
    }
    s = new std::string(ptr1);
    m_SmartSubstitutionTextComponents.push_back(s);

    // make the text buffer plenty big enough
    m_SmartSubstitutionTextBuffer = new char[length + m_SmartSubstitutionValues.size() * 64];

    m_SmartSubstitutionFlag = true;
}





