//
//  GFWMotor.hpp
//  RobotEnvironment
//
//  Created by Fletcher Wells on 2/16/19.
//  Copyright © 2019 Fletcher Wells. All rights reserved.
//

#ifndef GFWMotor_hpp
#define GFWMotor_hpp
#include <stdio.h>

#ifndef Classes_hpp
#include "Simulation.hpp"
#endif

/*
#ifndef GFWMotor__hpp
    #define GFWMotor__hpp
    #ifndef Simulation__hpp // is already included?
        #define Simulation__hpp // define this so we know it's include
        #include "Simulation.hpp"
    #endif
#endif
*/

// // this class currently assumes that the motor in use is a vex motor
class FEHMotor {
public:
    typedef enum {
        Motor0 = 0,
        Motor1,
        Motor2,
        Motor3
    } FEHMotorPort;
    
    FEHMotor( FEHMotorPort motorport, float max_voltage ) {
        port = motorport;
    }
    
    void Stop () {
        activePercent = 0;
        // (temporary)
        // simulatedVehicle.vel = Vector2 (-simulatedVehicle.vel.x, -simulatedVehicle.vel.y);
        // simulatedVehicle.vel = Vector2 (0, 0);
        UpdateWheel ();
    }
    void SetPercent (float percent) {
        activePercent = percent;
        if (percent == 0) {
            // (temporary)
            // simulatedVehicle.vel = Vector2 (-simulatedVehicle.vel.x, -simulatedVehicle.vel.y);
            simulatedVehicle.vel = Vector2 (0, 0);
            simulatedVehicle.angVel = 0;
        }
        UpdateWheel ();
    }
    
private:
    /*
    void SetPower (int power);
    float max_percent;
    FEHMotorPort motorport;
    char power;
    */
    int port = 0;
    float activePercent;
    
    void UpdateWheel () {
        simulatedVehicle.wheels [port].activePercent = activePercent / 100.0;
    }
};


#endif /* GFWMotor_hpp */
