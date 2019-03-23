//
//  GFWUtility.cpp
//  OpenGLTest
//
//  Created by Fletcher Wells on 12/10/18.
//  Copyright © 2018 Fletcher Wells. All rights reserved.
//

#include "GFWUtility.hpp"
#include <time.h>

#define TIME_SCALE_FACTOR 2 // used to speed up the time and get faster simulation results

float TimeNow () {
    return TIME_SCALE_FACTOR * glutGet(GLUT_ELAPSED_TIME) / 1000.0;
}

// note that this is a non FEH function
float TimeForRand () {
    return time (NULL);
}
