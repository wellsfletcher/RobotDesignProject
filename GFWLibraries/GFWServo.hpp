//
//  GFWServo.hpp
//  RobotEnvironment
//
//  Created by Fletcher Wells on 2/20/19.
//  Copyright © 2019 Fletcher Wells. All rights reserved.
//

#ifndef GFWServo_hpp
#define GFWServo_hpp
#include <stdio.h>
#endif /* GFWServo_hpp */


class FEHServo {
public:
    typedef enum {
        Servo0 = 0,
        Servo1,
        Servo2,
        Servo3,
        Servo4,
        Servo5,
        Servo6,
        Servo7
    } FEHServoPort;
    
    FEHServo ( FEHServoPort ) {
        /*
        servo_port = _servo;
        servo_min = 500;
        servo_max = 2500;
        
        // _position == -1 => servo is off
        _position = -1;
        */
    }
    void SetDegree (float degrees) {
        
    }
    void Calibrate ();
    void TouchCalibrate ();
    void Off ();
    void DigitalOn ();
    void DigitalOff ();
    void SetMax ( int max ) {
        
    }
    void SetMin ( int min ) {
        
    }
private:
    FEHServoPort servo_port;
    unsigned short servo_min;
    unsigned short servo_max;
    float _position;
};
