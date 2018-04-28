/*
 *  GLUtils.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 26/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef GLUtils_h
#define GLUtils_h

#include "PGDMath.h"

class StrokeFont;

struct Colour
{
    float r;
    float g;
    float b;
    float alpha;

    void SetColour(float rf, float gf, float bf, float af) { r = rf;  g = gf;  b = bf;  alpha = af; };
    void SetColour(Colour &c) { r = c.r;  g = c.g;  b = c.b;  alpha = c.alpha; };
};

enum ColourMap
{
    GreyColourMap,
    CoolColourMap,
    HotColourMap,
    HSVColourMap,
    JetColourMap,
    ParulaColourMap
};

class GLUtils
{
public:

    static void DrawAxes(float x, float y, float z, float ox = 0, float oy = 0, float oz = 0);
    static void OutputText(float x, float y, float z, char *text, double size, const float *matrix, const float *translation);
    static void SetColourFromMap(float index, ColourMap m, Colour *mappedColour, bool invert = false);
    static void DrawLine(float ix1, float iy1, float iz1, float ix2, float iy2, float iz2);
    static void SetDrawColour(float r, float g, float b, float a);

    static StrokeFont *strokeFont();
    static void setStrokeFont(StrokeFont *strokeFont);

protected:
    static StrokeFont *m_strokeFont;
};


#endif
