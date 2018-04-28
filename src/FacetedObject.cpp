/*
 *  FacetedObject.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 13/09/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include "FacetedObject.h"
#include "FacetedSphere.h"
#include "Face.h"
#include "DataFile.h"
#include "DebugControl.h"
#include "Util.h"

#include <ode/ode.h>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <cfloat>
#include <vector>
#include <list>
#include <string>
#include <sstream>

#if defined(USE_QT)
#include "GLUtils.h"
#include <GLWidget.h>
#endif

bool gDestinationOpenGL = true;
bool gDestinationPOVRay = false;
std::ofstream *gPOVRayFile = 0;
bool gDestinationOBJFile = false;
std::ofstream *gOBJFile = 0;

std::string gOBJName;
int gVertexOffset = 0;

// delayed draw control
bool gDelayedDraw = false;
std::list<FacetedObject *> gDelayedDrawList;

//#define DEBUG_SPEED

// create object
FacetedObject::FacetedObject()
{
    mNumVertices = 0;
    mNumVerticesAllocated = 0;
    mVertexList = 0;
    mNormalList = 0;
    mIndexList = 0;

    memset(m_DisplayPosition, 0, sizeof(dVector3));
    dRSetIdentity(m_DisplayRotation);

    m_UseRelativeOBJ = false;
    m_VerticesAsSpheresRadius = 0;

    m_AllocationIncrement = 8192;
    m_BadMesh = false;

#ifdef USE_QT
    m_glWidget = 0;
    m_BufferObjectsAllocated = false;
#endif
}

// destroy object
FacetedObject::~FacetedObject()
{
    if (mVertexList) delete mVertexList;
    if (mNormalList) delete mNormalList;
    if (mIndexList) delete mIndexList;
}

// parse an OBJ file to a FacetedObject
// returns true on error
bool FacetedObject::ParseOBJFile(const char *filename)
{
    DataFile theFile;
    if (theFile.ReadFile(filename)) return true;

#ifdef DEBUG_SPEED
    double start = Util::GetTime();
#endif
    bool st = ParseOBJFile(&theFile);
#ifdef DEBUG_SPEED
    double duration = Util::GetTime() - start;
    qDebug("%s %f\n", filename, duration);
#endif
    return st;
}

// parse an OBJ file to a FacetedObject
// returns true on error
bool FacetedObject::ParseOBJFile(DataFile *theFile)
{
    long bufferSizeLimit = theFile->GetSize() / 2; // if the file has any content it will have fewer lines than this
    char **linePtrs = new char *[bufferSizeLimit];
    long lineCount = DataFile::ReturnLines(theFile->GetRawData(), linePtrs, bufferSizeLimit);

    const int kBufferSize = 64000;
    char **tokens = new char *[kBufferSize];
    int numTokens;
    std::vector<pgd::Vector> vertexList;
    std::vector<std::vector<long> > faceList;
    std::vector<long> triFace(3);
    pgd::Vector vertex;
    double tri[9];
    long i, j;
    pgd::Vector min(DBL_MAX, DBL_MAX, DBL_MAX);
    pgd::Vector max(-DBL_MAX, -DBL_MAX, -DBL_MAX);
    long spaceToAllocate = 0;
    int biggestPolygon = 0;
    double v;

    // parse the lines
    for  (i = 0; i < lineCount; i++)
    {
        // vertices
        if (linePtrs[i][0] == 'v' && linePtrs[i][1] == ' ')
        {
            numTokens = DataFile::ReturnTokens(linePtrs[i], tokens, kBufferSize);
            if (numTokens > 3)
            {
                vertex.x = atof(tokens[1]);
                vertex.y = atof(tokens[2]);
                vertex.z = atof(tokens[3]);
                vertexList.push_back(vertex);

                if (gDebug == FacetedObjectDebug)
                {
                    min.x = MIN(min.x, vertex.x);
                    min.y = MIN(min.y, vertex.y);
                    min.z = MIN(min.z, vertex.z);
                    max.x = MAX(max.x, vertex.x);
                    max.y = MAX(max.y, vertex.y);
                    max.z = MAX(max.z, vertex.z);
                }
            }
        }

        // faces
        if (linePtrs[i][0] == 'f' && linePtrs[i][1] == ' ')
        {
            numTokens = DataFile::ReturnTokens(linePtrs[i], tokens, kBufferSize);
            if (numTokens == 4) // optimisation for triangles
            {
                triFace[0] = atol(tokens[1]) - 1;
                triFace[1] = atol(tokens[2]) - 1;
                triFace[2] = atol(tokens[3]) - 1;
                faceList.push_back(triFace);
                spaceToAllocate += 3;
                if (biggestPolygon < 3) biggestPolygon = 3;
                if (m_BadMesh) // currently duplicate the polygon but with reversed winding but this could be improved
                {
                    v = triFace[1];
                    triFace[1] = triFace[2];
                    triFace[2] = v;
                    faceList.push_back(triFace);
                    spaceToAllocate += 3;
                }

            }
            else
            {
                if (numTokens > 4)
                {
                    faceList.push_back(std::vector<long>(numTokens - 1));
                    // note obj files start at 1 not zero
                    for (j = 1; j < numTokens; j++)
                        faceList.back()[j - 1] = atol(tokens[j]) - 1;
                    spaceToAllocate += numTokens - 1;
                    if (biggestPolygon < numTokens - 1) biggestPolygon = numTokens - 1;
                    if (m_BadMesh) // currently duplicate the polygon but with reversed winding but this could be improved
                    {
                        faceList.push_back(std::vector<long>(numTokens - 1));
                        // note obj files start at 1 not zero
                        for (j = 1; j < numTokens; j++)
                            faceList.back()[j - 1] = atol(tokens[numTokens - j]) - 1;
                        spaceToAllocate += numTokens - 1;
                    }
                }
            }
        }
    }

    if (gDebug == FacetedObjectDebug)
        std::cerr << "ParseOBJFile:\tmin.x\t" << min.x << "\tmax.x\t" << max.x <<
                "\tmin.y\t" << min.y << "\tmax.y\t" << max.y <<
                "\tmin.z\t" << min.z << "\tmax.z\t" << max.z << "\n";


    if (m_VerticesAsSpheresRadius <= 0)
    {

        // fill out the display object
        m_AllocationIncrement = spaceToAllocate;
        if (biggestPolygon == 3) // optimise for triangular mesh
        {
            for (i = 0; i < (long)faceList.size(); i++)
            {
                for (j = 0; j < 3; j++)
                {
                    tri[j * 3] = vertexList[faceList[i][j]].x;
                    tri[j * 3 + 1] = vertexList[faceList[i][j]].y;
                    tri[j * 3 + 2] = vertexList[faceList[i][j]].z;
                }
                AddTriangle(tri);
            }
        }
        else
        {
            double *face = new double[biggestPolygon * 3];
            for (i = 0; i < (long)faceList.size(); i++)
            {
                if (faceList[i].size() == 3) // optimise for triangles
                {
                    for (j = 0; j < 3; j++)
                    {
                        tri[j * 3] = vertexList[faceList[i][j]].x;
                        tri[j * 3 + 1] = vertexList[faceList[i][j]].y;
                        tri[j * 3 + 2] = vertexList[faceList[i][j]].z;
                    }
                    AddTriangle(tri);
                }
                else
                {
                    for (j = 0; j < (long)faceList[i].size(); j++)
                    {
                        face[j * 3] = vertexList[faceList[i][j]].x;
                        face[j * 3 + 1] = vertexList[faceList[i][j]].y;
                        face[j * 3 + 2] = vertexList[faceList[i][j]].z;
                    }
                    AddPolygon(face, faceList[i].size());
                }
            }
            delete face;
        }
    }
    else
    {
        FacetedSphere masterSphere(m_VerticesAsSpheresRadius, 2);
        int nTri = masterSphere.GetNumTriangles();
        double *tri;
        pgd::Vector lastPos(0, 0, 0);
        m_AllocationIncrement = nTri * (int)vertexList.size();
        for (i = 0; i < (int)vertexList.size(); i++)
        {
            masterSphere.Move(vertexList[i].x - lastPos.x, vertexList[i].y - lastPos.y, vertexList[i].z - lastPos.z);
            lastPos = vertexList[i];
            for (j = 0; j < nTri; j++)
            {
                tri = masterSphere.GetTriangle(j);
                AddTriangle(tri);
            }
        }
    }

    // clear memory
    delete [] tokens;
    delete [] linePtrs;

    return false;
}

// write the object out as a POVRay string
// currently assumes all faces are triangles (call Triangulate if conversion is necessary)
void FacetedObject::WritePOVRay(std::ostringstream &theString)
{
    int i, j;
    double *vPtr;
    dVector3 prel, p, result;

    theString.precision(7); // should be plenty

    theString << "object {\n";
    theString << "  mesh {\n";

    // first faces
    for (i = 0; i < mNumVertices / 3; i++)
    {
        theString << "    triangle {\n";
        for (j = 0; j < 3; j++)
        {
            vPtr = mVertexList + i * 9 + j * 3;
            prel[0] = *vPtr++;
            prel[1] = *vPtr++;
            prel[2] = *vPtr;
            prel[3] = 0;
            dMULTIPLY0_331(p, m_DisplayRotation, prel);
            result[0] = p[0] + m_DisplayPosition[0];
            result[1] = p[1] + m_DisplayPosition[1];
            result[2] = p[2] + m_DisplayPosition[2];

            theString << "      <" << result[0] << "," << result[1] << "," << result[2] << ">\n";
        }
        theString << "    }\n";
    }

#ifdef USE_QT // this is only here because I don't define colours in the command line version - I might want to fix this
    // now colour
    theString << "    pigment {\n";
    theString << "      color rgbf<" << m_Colour.r << "," << m_Colour.g << "," << m_Colour.b <<"," << 1 - m_Colour.alpha << ">\n";
    theString << "    }\n";
    theString << "  }\n";
    theString << "}\n\n";
#endif
}

// Write a FacetedObject out as a OBJ
void FacetedObject::WriteOBJFile(std::ostringstream &out)
{
    int i, j;
    double *vPtr;
    dVector3 prel, p, result;
    static unsigned long counter = 0;

    out.precision(7); // should be plenty

    for (i = 0; i < (int)gOBJName.size(); i++)
        if (gOBJName[i] <= ' ') gOBJName[i] = '_';
    out << "o " << gOBJName << counter << "\n";
    counter++;

    if (m_UseRelativeOBJ)
    {
        // write out the vertices, faces, groups and objects
        // this is the relative version - inefficient but allows concatenation of objects
        for (i = 0; i < mNumVertices / 3; i++)
        {
            for (j = 0; j < 3; j++)
            {
                vPtr = mVertexList + i * 9 + j * 3;
                prel[0] = *vPtr++;
                prel[1] = *vPtr++;
                prel[2] = *vPtr;
                prel[3] = 0;
                dMULTIPLY0_331(p, m_DisplayRotation, prel);
                result[0] = p[0] + m_DisplayPosition[0];
                result[1] = p[1] + m_DisplayPosition[1];
                result[2] = p[2] + m_DisplayPosition[2];
                out << "v " << result[0] << " " << result[1] << " " << result[2] << "\n";
            }

            out << "f ";
            for (j = 0; j < 3; j++)
            {
                if (j == 3)
                    out << j - 3 << "\n";
                else
                    out << j - 3 << " ";
            }
        }
    }
    else
    {
        for (i = 0; i < mNumVertices / 3; i++)
        {
            for (j = 0; j < 3; j++)
            {
                vPtr = mVertexList + i * 9 + j * 3;
                prel[0] = *vPtr++;
                prel[1] = *vPtr++;
                prel[2] = *vPtr;
                prel[3] = 0;
                dMULTIPLY0_331(p, m_DisplayRotation, prel);
                result[0] = p[0] + m_DisplayPosition[0];
                result[1] = p[1] + m_DisplayPosition[1];
                result[2] = p[2] + m_DisplayPosition[2];
                out << "v " << result[0] << " " << result[1] << " " << result[2] << "\n";
            }
        }

        for (i = 0; i < mNumVertices / 3; i++)
        {
            out << "f ";
            for (j = 0; j < 3; j++)
            {
                // note this files vertex list start at 1 not zero
                if (j == 2)
                    out << *(mIndexList + i * 3 + j) + 1 + gVertexOffset << "\n";
                else
                    out << *(mIndexList + i * 3 + j) + 1 + gVertexOffset << " ";
            }
        }
        gVertexOffset += mNumVertices;
    }
}

void FacetedObject::Draw()
{
    if (m_Visible == false) return;

    if (gDelayedDraw)
    {
        gDelayedDrawList.push_back(this);
        return;
    }

    if (gDestinationOpenGL)
    {
#ifdef USE_QT
        if (m_glWidget)
        {
            if (m_BufferObjectsAllocated == false)
            {
                // order vertex data as x, y, z, xn, yn, zn
                int vertBufSize = mNumVertices * 3 * 2;
                GLfloat *vertBuf = new GLfloat[vertBufSize];
                GLfloat *vertBufPtr = vertBuf;
                double *vertexListPtr = mVertexList;
                double *normalListPtr = mNormalList;
                for (int i = 0; i < mNumVertices; i++)
                {
                    *vertBufPtr++ = *vertexListPtr++;
                    *vertBufPtr++ = *vertexListPtr++;
                    *vertBufPtr++ = *vertexListPtr++;
                    *vertBufPtr++ = *normalListPtr++;
                    *vertBufPtr++ = *normalListPtr++;
                    *vertBufPtr++ = *normalListPtr++;
                }

                // Setup our vertex buffer object.
                m_VBO.create();
                m_VBO.bind();
                m_VBO.allocate(vertBuf, vertBufSize * sizeof(GLfloat));

                delete [] vertBuf;
                m_BufferObjectsAllocated = true;
            }

            QMatrix4x4 model(
                    m_DisplayRotation[0], m_DisplayRotation[1], m_DisplayRotation[2],  m_DisplayPosition[0],
                    m_DisplayRotation[4], m_DisplayRotation[5], m_DisplayRotation[6],  m_DisplayPosition[1],
                    m_DisplayRotation[8], m_DisplayRotation[9], m_DisplayRotation[10], m_DisplayPosition[2],
                    0,                    0,                    0,                     1);

            // Store the vertex attribute bindings for the program.
            m_VBO.bind();
            QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
            f->glEnableVertexAttribArray(0);
            f->glEnableVertexAttribArray(1);
            f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), 0);
            f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));
            m_VBO.release();

            m_glWidget->facetedObjectShader()->bind();
            QMatrix4x4 modelView = m_glWidget->view() * model;
            m_glWidget->facetedObjectShader()->setUniformValue(m_glWidget->mvMatrixLoc(), modelView);
            QMatrix4x4 modelViewProjection = m_glWidget->proj() * modelView;
            m_glWidget->facetedObjectShader()->setUniformValue(m_glWidget->mvpMatrixLoc(), modelViewProjection);
            QMatrix3x3 normalMatrix = modelView.normalMatrix();
            m_glWidget->facetedObjectShader()->setUniformValue(m_glWidget->normalMatrixLoc(), normalMatrix);

            GLfloat r = m_Colour.r;
            GLfloat g = m_Colour.g;
            GLfloat b = m_Colour.b;
            GLfloat alpha = m_Colour.alpha;
            GLfloat ambientProportion = 0.2;
            GLfloat diffuseProportion = 0.8;
            GLfloat specularProportion = 0.3;
            GLfloat specularPower = 20;
            QVector4D ambient(r * ambientProportion, g * ambientProportion, b * ambientProportion, alpha);
            QVector4D diffuse(r * diffuseProportion, g * diffuseProportion, b * diffuseProportion, alpha);
            QVector4D specular(r * specularProportion, g * specularProportion, b * specularProportion, alpha);
            m_glWidget->facetedObjectShader()->setUniformValue(m_glWidget->ambientLoc(), ambient);
            m_glWidget->facetedObjectShader()->setUniformValue(m_glWidget->diffuseLoc(), diffuse);
            m_glWidget->facetedObjectShader()->setUniformValue(m_glWidget->specularLoc(), specular);
            m_glWidget->facetedObjectShader()->setUniformValue(m_glWidget->shininessLoc(), specularPower);

            glDrawArrays(GL_TRIANGLES, 0, mNumVertices);
        }
#endif
    }

    else if (gDestinationPOVRay)
    {
        std::ostringstream theString;
        WritePOVRay(theString);
        if (gPOVRayFile) (*gPOVRayFile) << theString.str();
        else std::cout << theString.str();
    }

    else if (gDestinationOBJFile)
    {
        std::ostringstream theString;
        WriteOBJFile(theString);
        if (gOBJFile) (*gOBJFile) << theString.str();
        else std::cout << theString.str();
    }
}

void FacetedObject::SetDisplayPosition(double x, double y, double z)
{
    m_DisplayPosition[0] = x;
    m_DisplayPosition[1] = y;
    m_DisplayPosition[2] = z;
}

void FacetedObject::SetDisplayRotation(const dMatrix3 R, bool fast)
{
    if (fast)
    {
        memcpy(m_DisplayRotation, R, sizeof(dMatrix3));
    }
    else
    {
        dQuaternion q;
        dRtoQ (R, q);
        dNormalize4 (q);
        dQtoR (q, m_DisplayRotation);
    }
}

void FacetedObject::SetDisplayRotationFromQuaternion(const dQuaternion q, bool fast)
{
    if (fast == false)
    {
        dQuaternion qq;
        memcpy(qq, q, sizeof(dQuaternion));
        dNormalize4 (qq);
        dQtoR(qq, m_DisplayRotation);
    }
    else
        dQtoR(q, m_DisplayRotation);
}

// this routine rotates the Z axis to point in a specified direction
void FacetedObject::SetDisplayRotationFromAxis(double x, double y, double z, bool fast)
{
    // calculate the rotation needed to get the axis pointing the right way
    dVector3 axis;
    axis[0] = x;
    axis[1] = y;
    axis[2] = z;
    if (fast == false) dNormalize3(axis);
    dVector3 p, q;
    // calculate 2 perpendicular vectors
    dPlaneSpace(axis, p, q);
    // assemble the matrix
    m_DisplayRotation[3] = m_DisplayRotation[7] = m_DisplayRotation[11] = 0;

    m_DisplayRotation[0] =    p[0]; m_DisplayRotation[4] =    p[1]; m_DisplayRotation[8] =     p[2];
    m_DisplayRotation[1] =    q[0]; m_DisplayRotation[5] =    q[1]; m_DisplayRotation[9] =     q[2];
    m_DisplayRotation[2] = axis[0]; m_DisplayRotation[6] = axis[1]; m_DisplayRotation[10] = axis[2];
}

// utility to calculate a face normal
// this assumes anticlockwise winding
void FacetedObject::ComputeFaceNormal(const double *v1, const double *v2, const double *v3, double normal[3])
{
    double a[3], b[3];

    // calculate in plane vectors
    a[0] = v2[0] - v1[0];
    a[1] = v2[1] - v1[1];
    a[2] = v2[2] - v1[2];
    b[0] = v3[0] - v1[0];
    b[1] = v3[1] - v1[1];
    b[2] = v3[2] - v1[2];

    // cross(a, b, normal);
    normal[0] = a[1] * b[2] - a[2] * b[1];
    normal[1] = a[2] * b[0] - a[0] * b[2];
    normal[2] = a[0] * b[1] - a[1] * b[0];

    // normalize(normal);
    double norm = sqrt(normal[0] * normal[0] +
                      normal[1] * normal[1] +
                      normal[2] * normal[2]);

    if (norm > 0.0)
    {
        normal[0] /= norm;
        normal[1] /= norm;
        normal[2] /= norm;
    }
}

// move the object
// note this must be used before first draw call
void FacetedObject::Move(double x, double y, double z)
{
    for (int i = 0; i < mNumVertices; i++)
    {
        mVertexList[i * 3] += x;
        mVertexList[i * 3 + 1] += y;
        mVertexList[i * 3 + 2] += z;
    }
}

// scale the object
// note this must be used before first draw call
void FacetedObject::Scale(double x, double y, double z)
{
    for (int i = 0; i < mNumVertices; i++)
    {
        mVertexList[i * 3] *= x;
        mVertexList[i * 3 + 1] *= y;
        mVertexList[i * 3 + 2] *= z;
    }
}

// this routine triangulates the polygon and calls AddTriangle to do the actual data adding
// vertices are a packed list of floating point numbers
// x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4
void FacetedObject::AddPolygon(const double *vertices, int nSides)
{
    // add faces as triangles
    double triangle[9];
    triangle[0] = vertices[0];
    triangle[1] = vertices[1];
    triangle[2] = vertices[2];
    for (int j = 2; j < nSides; j++)
    {
        triangle[3] = vertices[(j - 1) * 3];
        triangle[4] = vertices[(j - 1) * 3 + 1];
        triangle[5] = vertices[(j - 1) * 3 + 2];
        triangle[6] = vertices[(j * 3)];
        triangle[7] = vertices[(j * 3) + 1];
        triangle[8] = vertices[(j * 3) + 2];
        AddTriangle(triangle);
    }
}


// this is the only routine that actually adds data to the facetted object
// it gets called by add polygon
// vertices is a packed list of floating point numbers
// x1, y1, z1, x2, y2, z2, x3, y3, z3
void FacetedObject::AddTriangle(const double *vertices)
{
    int newNumVertices = mNumVertices + 3;
    if (newNumVertices > mNumVerticesAllocated)
    {
        AllocateMemory(mNumVerticesAllocated + m_AllocationIncrement);
        m_AllocationIncrement *= 2;
    }
    memcpy(mVertexList + mNumVertices * 3, vertices, sizeof(double) * 9);

    // now calculate the normals
    double normal[3];
    ComputeFaceNormal(vertices, vertices + 3, vertices + 6, normal);
    memcpy(mNormalList + mNumVertices * 3, normal, sizeof(double) * 3);
    memcpy(mNormalList + mNumVertices * 3 + 3, normal, sizeof(double) * 3);
    memcpy(mNormalList + mNumVertices * 3 + 6, normal, sizeof(double) * 3);

    // and finally the indices
    mIndexList[mNumVertices] = mNumVertices;
    mNumVertices++;
    mIndexList[mNumVertices] = mNumVertices;
    mNumVertices++;
    mIndexList[mNumVertices] = mNumVertices;
    mNumVertices++;
}

// this routine handles the memory allocation
void FacetedObject::AllocateMemory(int allocation)
{
    if (allocation > mNumVerticesAllocated)
    {
        mNumVerticesAllocated = allocation;
        double *newVertexList = new double[mNumVerticesAllocated * 3];
        double *newNormalList = new double[mNumVerticesAllocated * 3];
        int *newIndexList = new int[mNumVerticesAllocated];
        if (mVertexList)
        {
            memcpy(newVertexList, mVertexList, sizeof(double) * mNumVertices * 3);
            delete [] mVertexList;
            memcpy(newNormalList, mNormalList, sizeof(double) * mNumVertices * 3);
            delete [] mNormalList;
            memcpy(newIndexList, mIndexList, sizeof(int) * mNumVertices);
            delete [] mIndexList;
        }
        mVertexList = newVertexList;
        mNormalList = newNormalList;
        mIndexList = newIndexList;
    }
}

// return an ODE style trimesh
// note memory is allocated by this routine and will need to be released elsewhere
void FacetedObject::CalculateTrimesh(double **vertices, int *numVertices, int *vertexStride, dTriIndex **triIndexes, int *numTriIndexes, int *triStride)
{
    int i;
    *vertexStride = 3 * sizeof(double);
    *triStride = 3 * sizeof(dTriIndex);

    *numVertices = mNumVertices;
    *numTriIndexes = mNumVertices;

    *vertices = new double[mNumVertices * 3];
    *triIndexes = new dTriIndex[mNumVertices];

    for (i = 0; i < mNumVertices; i++)
    {
        (*vertices)[i * 3] = mVertexList[i * 3];
        (*vertices)[i * 3 + 1] = mVertexList[i * 3 + 1];
        (*vertices)[i * 3 + 2] = mVertexList[i * 3 + 2];
    }

    for (i = 0; i < mNumVertices; i++)
    {
        (*triIndexes)[i] = mIndexList[i];
    }
}

// return an ODE style trimesh
// note memory is allocated by this routine and will need to be released elsewhere
void FacetedObject::CalculateTrimesh(float **vertices, int *numVertices, int *vertexStride, dTriIndex **triIndexes, int *numTriIndexes, int *triStride)
{
    int i;
    *vertexStride = 3 * sizeof(float);
    *triStride = 3 * sizeof(dTriIndex);

    *numVertices = mNumVertices;
    *numTriIndexes = mNumVertices;

    *vertices = new float[mNumVertices * 3];
    *triIndexes = new dTriIndex[mNumVertices];

    for (i = 0; i < mNumVertices; i++)
    {
        (*vertices)[i * 3] = mVertexList[i * 3];
        (*vertices)[i * 3 + 1] = mVertexList[i * 3 + 1];
        (*vertices)[i * 3 + 2] = mVertexList[i * 3 + 2];
    }

    for (i = 0; i < mNumVertices; i++)
    {
        (*triIndexes)[i] = mIndexList[i];
    }
}

// calculate mass properties
// based on dMassSetTrimesh
/*
 * dMassSetTrimesh, implementation by Gero Mueller.
 * Based on Brian Mirtich, "Fast and Accurate Computation of
 * Polyhedral Mass Properties," journal of graphics tools, volume 1,
 * number 2, 1996.
 */

#define	SQR(x)			((x)*(x))						//!< Returns x square
#define	CUBE(x)			((x)*(x)*(x))					//!< Returns x cube

#define _I(i,j) I[(i)*4+(j)]

void FacetedObject::CalculateMassProperties(dMass *m, double density, bool clockwise)
{
    dMassSetZero (m);

    // assumes anticlockwise winding

    unsigned int triangles = mNumVertices / 3;

    double nx, ny, nz;
    unsigned int i, j, A, B, C;
    // face integrals
    double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;

    // projection integrals
    double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

    double T0 = 0;
    double T1[3] = {0., 0., 0.};
    double T2[3] = {0., 0., 0.};
    double TP[3] = {0., 0., 0.};

    dVector3 v[3];
    for( i = 0; i < triangles; i++ )
    {
        if (clockwise == false)
        {
            for (j = 0; j < 3; j++)
            {
                v[j][0] = mVertexList[i * 9 + j * 3];
                v[j][1] = mVertexList[i * 9 + j * 3 + 1];
                v[j][2] = mVertexList[i * 9 + j * 3 + 2];
            }
        }
        else
        {
            for (j = 0; j < 3; j++)
            {
                v[j][2] = mVertexList[i * 9 + j * 3];
                v[j][1] = mVertexList[i * 9 + j * 3 + 1];
                v[j][0] = mVertexList[i * 9 + j * 3 + 2];
            }
        }

        dVector3 n, a, b;
        dOP( a, -, v[1], v[0] );
        dOP( b, -, v[2], v[0] );
        dCROSS( n, =, b, a );
        nx = fabs(n[0]);
        ny = fabs(n[1]);
        nz = fabs(n[2]);

        if( nx > ny && nx > nz )
            C = 0;
        else
            C = (ny > nz) ? 1 : 2;

        // Even though all triangles might be initially valid,
        // a triangle may degenerate into a segment after applying
        // space transformation.
        if (n[C] != REAL(0.0))
        {
            A = (C + 1) % 3;
            B = (A + 1) % 3;

            // calculate face integrals
            {
                double w;
                double k1, k2, k3, k4;

                //compProjectionIntegrals(f);
                {
                    double a0, a1, da;
                    double b0, b1, db;
                    double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
                    double a1_2, a1_3, b1_2, b1_3;
                    double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
                    double Cab, Kab, Caab, Kaab, Cabb, Kabb;

                    P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

                    for( j = 0; j < 3; j++)
                    {
                        switch(j)
                        {
                        case 0:
                            a0 = v[0][A];
                            b0 = v[0][B];
                            a1 = v[1][A];
                            b1 = v[1][B];
                            break;
                        case 1:
                            a0 = v[1][A];
                            b0 = v[1][B];
                            a1 = v[2][A];
                            b1 = v[2][B];
                            break;
                        case 2:
                            a0 = v[2][A];
                            b0 = v[2][B];
                            a1 = v[0][A];
                            b1 = v[0][B];
                            break;
                        }
                        da = a1 - a0;
                        db = b1 - b0;
                        a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
                        b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
                        a1_2 = a1 * a1; a1_3 = a1_2 * a1;
                        b1_2 = b1 * b1; b1_3 = b1_2 * b1;

                        C1 = a1 + a0;
                        Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
                        Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
                        Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
                        Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
                        Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
                        Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

                        P1 += db*C1;
                        Pa += db*Ca;
                        Paa += db*Caa;
                        Paaa += db*Caaa;
                        Pb += da*Cb;
                        Pbb += da*Cbb;
                        Pbbb += da*Cbbb;
                        Pab += db*(b1*Cab + b0*Kab);
                        Paab += db*(b1*Caab + b0*Kaab);
                        Pabb += da*(a1*Cabb + a0*Kabb);
                    }

                    P1 /= 2.0;
                    Pa /= 6.0;
                    Paa /= 12.0;
                    Paaa /= 20.0;
                    Pb /= -6.0;
                    Pbb /= -12.0;
                    Pbbb /= -20.0;
                    Pab /= 24.0;
                    Paab /= 60.0;
                    Pabb /= -60.0;
                }

                w = - dDOT(n, v[0]);

                k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

                Fa = k1 * Pa;
                Fb = k1 * Pb;
                Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

                Faa = k1 * Paa;
                Fbb = k1 * Pbb;
                Fcc = k3 * (SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb +
                            w*(2*(n[A]*Pa + n[B]*Pb) + w*P1));

                Faaa = k1 * Paaa;
                Fbbb = k1 * Pbbb;
                Fccc = -k4 * (CUBE(n[A])*Paaa + 3*SQR(n[A])*n[B]*Paab
                              + 3*n[A]*SQR(n[B])*Pabb + CUBE(n[B])*Pbbb
                              + 3*w*(SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb)
                              + w*w*(3*(n[A]*Pa + n[B]*Pb) + w*P1));

                Faab = k1 * Paab;
                Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
                Fcca = k3 * (SQR(n[A])*Paaa + 2*n[A]*n[B]*Paab + SQR(n[B])*Pabb
                             + w*(2*(n[A]*Paa + n[B]*Pab) + w*Pa));
            }


            T0 += n[0] * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));

            T1[A] += n[A] * Faa;
            T1[B] += n[B] * Fbb;
            T1[C] += n[C] * Fcc;
            T2[A] += n[A] * Faaa;
            T2[B] += n[B] * Fbbb;
            T2[C] += n[C] * Fccc;
            TP[A] += n[A] * Faab;
            TP[B] += n[B] * Fbbc;
            TP[C] += n[C] * Fcca;
        }
    }

    T1[0] /= 2; T1[1] /= 2; T1[2] /= 2;
    T2[0] /= 3; T2[1] /= 3; T2[2] /= 3;
    TP[0] /= 2; TP[1] /= 2; TP[2] /= 2;

    m->mass = density * T0;
    m->_I(0,0) = density * (T2[1] + T2[2]);
    m->_I(1,1) = density * (T2[2] + T2[0]);
    m->_I(2,2) = density * (T2[0] + T2[1]);
    m->_I(0,1) = - density * TP[0];
    m->_I(1,0) = - density * TP[0];
    m->_I(2,1) = - density * TP[1];
    m->_I(1,2) = - density * TP[1];
    m->_I(2,0) = - density * TP[2];
    m->_I(0,2) = - density * TP[2];

    m->c[0] = T1[0] / T0;
    m->c[1] = T1[1] / T0;
    m->c[2] = T1[2] / T0;

}

#ifdef USE_QT
GLWidget *FacetedObject::glWidget() const
{
    return m_glWidget;
}

void FacetedObject::setGlWidget(GLWidget *glWidget)
{
    m_glWidget = glWidget;
}
#endif

// reverse the face winding
void FacetedObject::ReverseWinding()
{
    double t;
    int numTriangles = mNumVertices / 3;
    int i, j;
    for (i = 0; i < numTriangles; i++)
    {
        for (j = 0; j < 3; j++)
        {
            t = mVertexList[i * 9 + 3 + j];
            mVertexList[i * 9 + 3 + j] = mVertexList[i * 9 + 6 + j];
            mVertexList[i * 9 + 6 + j] = t;
        }
    }
    for (i = 0; i < mNumVertices * 3; i++) mNormalList[i] = -mNormalList[i];
}

// add the faces from one faceted object to another
void FacetedObject::AddFacetedObject(FacetedObject *object, bool useDisplayRotation)
{
    int numTriangles = object->GetNumTriangles();
    double *triangle, *p1;
    double triangle2[9];
    dVector3 v1, v1r;

    if (useDisplayRotation)
    {
        for (int i = 0; i < numTriangles; i++)
        {
            triangle = object->GetTriangle(i);
            for (int j =0; j < 3; j++)
            {
                p1 = triangle + 3 * j;
                v1[0] = p1[0];
                v1[1] = p1[1];
                v1[2] = p1[2];
                v1[3] = 0;
                dMULTIPLY0_331(v1r, m_DisplayRotation, v1);
                p1 = triangle2 + 3 * j;
                p1[0] = v1r[0] + m_DisplayPosition[0];
                p1[1] = v1r[1] + m_DisplayPosition[1];
                p1[2] = v1r[2] + m_DisplayPosition[2];
            }
            AddTriangle(triangle2);
        }
    }
    else
    {
        for (int i = 0; i < numTriangles; i++)
        {
            triangle = object->GetTriangle(i);
            AddTriangle(triangle);
        }
    }
}
