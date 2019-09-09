//
//  GFWUtility.cpp
//  OpenGLTest
//
//  Created by Fletcher Wells on 12/10/18.
//  Copyright © 2018 Fletcher Wells. All rights reserved.
//

#include "GFWUtility.hpp"
#include <time.h>

// #define TIME_SCALE_FACTOR 3 // used to speed up the time and get faster simulation results
float TIME_SCALE_FACTOR = 3.0;
float timeWhenUpdated = 0;
float alteredTime = 0; // the current time with any alterations due to changing the time scale (TIME_SCALE_FACTOR)


float TimeNow () {    
    float theTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    float timeSinceUpdate = theTime - timeWhenUpdated;
    
    alteredTime += TIME_SCALE_FACTOR * timeSinceUpdate;
    
    timeWhenUpdated = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    return alteredTime;
}

// note that this is a non FEH function
float TimeForRand () {
    return time (NULL);
}

