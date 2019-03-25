//
//  GFWIO.hpp
//  RobotEnvironment
//
//  Created by Fletcher Wells on 2/19/19.
//  Copyright © 2019 Fletcher Wells. All rights reserved.
//

#ifndef GFWIO_hpp
#define GFWIO_hpp
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

class FEHIO {
public:
    typedef enum {
        P0_0 = 0,
        P0_1,
        P0_2,
        P0_3,
        P0_4,
        P0_5,
        P0_6,
        P0_7,
        P1_0,
        P1_1,
        P1_2,
        P1_3,
        P1_4,
        P1_5,
        P1_6,
        P1_7,
        P2_0,
        P2_1,
        P2_2,
        P2_3,
        P2_4,
        P2_5,
        P2_6,
        P2_7,
        P3_0,
        P3_1,
        P3_2,
        P3_3,
        P3_4,
        P3_5,
        P3_6,
        P3_7,
        BATTERY_VOLTAGE
    }FEHIOPin;
    
    typedef enum
    {
        Bank0 = 0,
        Bank1,
        Bank2,
        Bank3
    }FEHIOPort;
    
    typedef enum
    {
        RisingEdge = 0x09,
        FallingEdge = 0x0A,
        EitherEdge = 0x0B
    }FEHIOInterruptTrigger;
};


// Begin Class Declarations for Pin Types
class DigitalInputPin {
public:
    DigitalInputPin ( FEHIO::FEHIOPin pin ) {
        timerStarted = false;
        _pin = pin;
    }
    /*
    bool Value () { // I just realized this probably won't work
        // if the timer has not started then start the timer
        if (!timerStarted) {
            timerStarted = true;
            timeWhenTimerStarted = GetTheTime ();
        }
        // if the alloted time has passed then make the timer not have started; also return "false" to indicate the -bump switch- has activated
        bool theValue = GetTheTime() - timeWhenTimerStarted > TIME_UNTIL_ACTIVATION;
        if (theValue) {
            timerStarted = false;
        }
        
        return !theValue;
    }
    */
    bool Value () { // I just realized this probably won't work
        bool simulatedValue = false;
        int bumpID = 0;
        
        switch (_pin) {
            case FEHIO::P1_0:
                bumpID = 0;
                break;
            case FEHIO::P1_1:
                bumpID = 1;
                break;
            case FEHIO::P1_2:
                bumpID = 2;
                break;
            case FEHIO::P1_3:
                bumpID = 3;
                break;
            case FEHIO::P1_5:
                bumpID = 4;
                break;
            case FEHIO::P1_6:
                bumpID = 5;
                break;
                
            default:
                // error; incorrect bump switch
                bumpID = 0;
                break;
        }
        
        simulatedValue = simulatedVehicle.bumps [bumpID].Value ();
        
        /*
        if (!simulatedValue) {
            cout << bumpID << endl;
        }
        */
        
        return simulatedValue;
    }
    /*
    bool Value () {
        return false;
    }
    */
    
    friend class ButtonBoard;
    FEHIO::FEHIOPin _pin;
    
private:
    
    DigitalInputPin ();
    void Initialize ( FEHIO::FEHIOPin pin );
    
    // simulation bois
    float GetTheTime () { // equivalent to TimeNow ()
        return glutGet (GLUT_ELAPSED_TIME) / 1000.0;
    }
    constexpr static float TIME_UNTIL_ACTIVATION = 1.0;
    float timeWhenTimerStarted;
    bool timerStarted;
};


class AnalogInputPin {
protected:
    FEHIO::FEHIOPin pin;
    // static tADC_Config Master_Adc_Config;
    // static tADC_Config Encoder_Adc_Config;
    
public:
    AnalogInputPin ( FEHIO::FEHIOPin ) {
        srand (TimeNow ());
    }
    float Value() {
        Update ();
        return value;
    }

    // static void InitADCs();
private:
    float value = 0;
    int VARIANCE = 2000;
    float REFRESH_RATE = .5; // 1.0
    float timeWhenTimerStarted = 0;
    
    void Update () {
        if (TimeNow () - timeWhenTimerStarted > REFRESH_RATE) {
            timeWhenTimerStarted = TimeNow ();
            
            value = 1.0;
            value += (rand () % VARIANCE / 1000.0 - (VARIANCE / 2000.0));
        }
    }
};



#endif /* GFWIO_hpp */
