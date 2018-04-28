/*
 *  FacetedObject.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 13/09/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef FacetedObject_h
#define FacetedObject_h

#include "NamedObject.h"

#include <ode/ode.h>

#ifdef USE_QT
class GLWidget;
#include <QOpenGLBuffer>
#endif

class DataFile;
class TrimeshGeom;
class ostringstream;

class FacetedObject: public NamedObject
{
public:
    FacetedObject();
    ~FacetedObject();

    bool ParseOBJFile(const char *filename);
    bool ParseOBJFile(DataFile *theFile);

    virtual void WritePOVRay(std::ostringstream &theString);
    virtual void WriteOBJFile(std::ostringstream &out);

    int GetNumVertices() { return mNumVertices; }
    double *GetVertex(int i) { return mVertexList + (3 * i); }
    double *GetNormal(int i) { return mNormalList + (3 * i); }
    double *GetVertexList() { return mVertexList; }
    double *GetNormalList() { return mNormalList; }
    int *GetIndexList() { return mIndexList; }

    void AddPolygon(const double *vertices, int nSides);
    void AddTriangle(const double *vertices);
    void AddFacetedObject(FacetedObject *object, bool useDisplayRotation);

    int GetNumTriangles() { return mNumVertices / 3; }
    double *GetTriangle(int i) { return mVertexList + (9 * i); }

    virtual void Draw();
    void SetDisplayPosition(double x, double y, double z);
    void SetDisplayRotation(const dMatrix3 R, bool fast = true);
    void SetDisplayRotationFromQuaternion(const dQuaternion q, bool fast = true);
    void SetDisplayRotationFromAxis(double x, double y, double z, bool fast = true);

    const double *GetDisplayPosition()  { return m_DisplayPosition; }
    const double *GetDisplayRotation()  { return m_DisplayRotation; }

    // static utilities
    static void ComputeFaceNormal(const double *v1, const double *v2, const double *v3, double normal[3]);

    // manipulation functions
    void Move(double x, double y, double z);
    void Scale(double x, double y, double z);
    void Mirror(bool x, bool y, bool z);
    void SwapAxes(int axis1, int axis2);
    void RotateAxes(int axis0, int axis1, int axis2);

    // utility
    void ReverseWinding();
    void AllocateMemory(int allocation);
    void SetVerticesAsSpheresRadius(double verticesAsSpheresRadius) { m_VerticesAsSpheresRadius = verticesAsSpheresRadius; }

    // ODE link
    void CalculateTrimesh(double **vertices, int *numVertices, int *vertexStride, dTriIndex **triIndexes, int *numTriIndexes, int *triStride);
    void CalculateTrimesh(float **vertices, int *numVertices, int *vertexStride, dTriIndex **triIndexes, int *numTriIndexes, int *triStride);
    void CalculateMassProperties(dMass *m, double density, bool clockwise);

    void SetBadMesh(bool v) { m_BadMesh = v; }

#ifdef USE_QT
    GLWidget *glWidget() const;
    void setGlWidget(GLWidget *glWidget);
#endif

protected:

    int mNumVertices;
    int mNumVerticesAllocated;
    double *mVertexList;
    double *mNormalList;
    int *mIndexList;

    dVector3 m_DisplayPosition;
    dMatrix3 m_DisplayRotation;

    bool m_UseRelativeOBJ;

    double m_VerticesAsSpheresRadius;

    bool m_BadMesh;

#ifdef USE_QT
    GLWidget *m_glWidget;
    QOpenGLBuffer m_VBO;
    bool m_BufferObjectsAllocated;
#endif

    int m_AllocationIncrement;
};

#endif
