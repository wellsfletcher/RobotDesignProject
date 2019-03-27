//
//  GFWRPS.hpp
//  RobotEnvironment
//
//  Created by Fletcher Wells on 3/3/19.
//  Copyright © 2019 Fletcher Wells. All rights reserved.
//

#ifndef GFWRPS_hpp
#define GFWRPS_hpp

#include <stdio.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <OpenGL/glext.h>

#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <time.h>

#ifndef Classes_hpp
#include "Simulation.hpp"
#endif

#ifndef GFWUtility_hpp
#include "GFWUtility.hpp"
#endif


class FEHRPS
{
public:
    
    FEHRPS () {
        angleAtStart = 0;
        srand (TimeNow ());
        timeWhenTimerStarted = TimeNow ();
    }
    float RPS_X;
    float RPS_Y;
    float RPS_Heading;
    
    float course_height = 6*12;
    
    float x_offset = 0.0; // the displacement in which the returned x-value is off by
    float y_offset = 0.0; // the displacement in which the returned y-value is off by
    float x_scale = 1.0; // the scale by which the returned x-value is off by
    float y_scale = 1.0; // the scale by which the returned y-value is off by
    
    // Creates a menu to allow you to pick the correct region
    // Assumes ButtonBoard is plugged into Bank3
    // Right button increments region
    // Left button decrements region
    // Middle button selects region
    void InitializeMenu () {
        InitializeTouchMenu ();
    }
    
    void InitializeTouchMenu () {
        
    }
    
    // Manually pick and configure a region
    // int region => { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
    // char region => { a, b, c, d, e, f, g, h, i, j, k, l } || { A, B, C, D, E, F, G, H, I, J, K, L }
    void Initialize( int region );
    void Initialize( char region );
    
    // return the current course number { 1, 2, 3 }
    unsigned char CurrentCourse () {
        return 1;
    }
    
    // returns the letter of the current region { A, B, C, D, E, F, G, H, I, J, K, L }
    char CurrentRegionLetter () {
        // angleAtStart = simulatedVehicle.dir.getAngle (); // secretly initialize the starting angle
        // angleAtStart = 255;
        return 'A';
    }
    
    // returns the number of the current course { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
    int CurrentRegion () {
        return 0;
    }
    
    // Objective functions:
    int DeadzoneTimeLeft () {
        return 69;
    }
    
    // returns the match time in seconds
    int Time () {
        return TimeNow ();
    }
    
    unsigned char WaitForPacket();
    int WaitForPacketDebug(int *packetsFound, int *packetsLost,int *lastFoundPacketTime);
    
    float X () {
        Update ();
        return RPS_X;
    }
    float Y () {
        Update ();
        return RPS_Y;
    }
    /*
    float X () {
        float x0 = simulatedVehicle.pos.x * x_scale + x_offset;
        x0 = (((int)(x0)) / 2) * 2;
        return x0;
    }
    float Y () {
        float y0 = (simulatedVehicle.pos.y + course_height) * y_scale + y_offset;
        y0 = (((int)(y0)) / 2) * 2;
        return y0;
    }
    */
    float Heading () {
        Update ();
        return RPS_Heading;
    }
    
    // simulation
    /*
    float GetTheTime () { // equivalent to TimeNow ()
        return glutGet (GLUT_ELAPSED_TIME) / 1000.0;
    }
    */
    void Update () {
        if (TimeNow () - timeWhenTimerStarted > REFRESH_RATE) {
            timeWhenTimerStarted = TimeNow ();
            
            RPS_X = simulatedVehicle.pos.x * x_scale + x_offset;
            RPS_Y = (simulatedVehicle.pos.y + course_height) * y_scale + y_offset;
            
            if (RPS_Y > DEADZONE_START_Y_POS) {
                SetOutputToDeadzone ();
            } else {
                RPS_Heading = Vector2::CapDegrees (simulatedVehicle.dir.getAngle () - angleAtStart); // angle at start is 0
                
                if (rand () % RANDOM_FACTOR == 0) {
                    // MakeOutputUndefined ();
                }
            }
            
        }
    }
    
private:
    // FEHXBee _xbee;
    int _region;
    float angleAtStart;
    
    float REFRESH_RATE = 0.4; // 0.5 // 0.8
    float timeWhenTimerStarted = 0;
    int RANDOM_FACTOR = 200;
    
    float DEADZONE_START_Y_POS = 54;
    
    void MakeOutputUndefined () {
        RPS_X = -1;
        RPS_Y = -1;
        RPS_Heading = -1;
    }
    
    void SetOutputToDeadzone () {
        RPS_X = -2;
        RPS_Y = -2;
        // RPS_Heading = 0;
    }
};

extern FEHRPS RPS;





#endif /* GFWRPS_hpp */

