//
//  GFWFunctions.hpp
//  RobotEnvironment
//
//  Created by Fletcher Wells on 2/6/19.
//  Copyright © 2019 Fletcher Wells. All rights reserved.
//

#ifndef GFWFunctions_hpp
#define GFWFunctions_hpp
#include <stdio.h>
#endif /* GFWLCD_hpp */

//#include "GL/freeglut.h"
//#include "GL/glut.h"

#include <cstdlib>                      // standard definitions
#include <iostream>                     // C++ I/O
#include <cstdio>                       // C I/O (for sprintf)
#include <cmath>                        // standard definitions

using namespace std;                    // make std accessible


struct RGB {
    float r;
    float g;
    float b;
};
typedef struct RGB RGB;

int ConvertColor (float r, float g, float b);
int ConvertColor (RGB rgbColor);
RGB ConvertColor (int hexColor);
int ConvertToShadow (int hexColor, int shadowColor, float shadowAlpha);
