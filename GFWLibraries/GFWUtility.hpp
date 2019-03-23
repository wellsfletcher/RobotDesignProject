#ifndef GFWUtility_hpp
#define GFWUtility_hpp
#include <stdio.h>


#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <OpenGL/glext.h>

#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

//#include "GL/freeglut.h"
//#include "GL/glut.h"

#include <cstdlib>                      // standard definitions
#include <iostream>                     // C++ I/O
#include <cstdio>                       // C I/O (for sprintf)
#include <cmath>                        // standard definitions

using namespace std;                    // make std accessible



float TimeNow ();
float TimeForRand ();


#endif /* GFWUtility_hpp */
