#define IS_SIMULATION 0 // also have to define this variable in Simulation.hpp and Classes.hpp
#define SUB_SIMULATION_ENABLED 1
// #define FULL_SIMULATION 1
#if IS_SIMULATION // if this is the simulation, then this chunck of code is used. Otherwise, this code is ignored.
/*
 
 
 
 SIMULATION only stuff
 
 
 
 */






#include "GFWFunctions.hpp"
#include "GFWMotor.hpp"
#include "GFWIO.hpp"
#include "GFWServo.hpp"
#include "GFWRPS.hpp"

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <OpenGL/glext.h>

#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <math.h>
#include <vector>
using namespace std;

#include <cstdlib>                      // standard definitions
#include <iostream>                     // C++ I/O
#include <cstdio>                       // C I/O (for sprintf)
#include <cmath>                        // standard definitions



void Init ();
void mainLoop ();
void DrawEverything ();
bool leftKeyInp;
bool upKeyInp;
bool rightKeyInp;
bool downKeyInp;

#define CALLBACK_DELAY .0001 // in seconds
#define UPDATE_INTERVAL 0.005
#define TIME_SCALE_ALTERATION_MODE 1 // if enabled, allows the time scale to be altered using the keyboard
GLint TIMER_DELAY = 5; // in milleseconds

void Reshape(int w, int h) {
    glViewport (0, 0, w, h);                    // update the viewport
    glMatrixMode(GL_PROJECTION);                // update projection
    glLoadIdentity();
    gluOrtho2D(0.0, 320, 0.0, 240);             // map unit square to viewport
    glMatrixMode(GL_MODELVIEW);
}

void Display (void) {                          // display callback
    DrawEverything ();
    glFlush();
}

void PassiveMouseV2 (int x, int y) {             // updates upon mouse movement when there are mouse clicks/inputs
    if (LCD.touchState) {
        LCD.touchX = x/ZOOM_SCALE;
        LCD.touchY = y/ZOOM_SCALE;
    }
}

void Mouse (int buttonState, int clickState, int x, int y) {      // mouse click callback ... only updates upon mouse click
    if (clickState == GLUT_DOWN && buttonState == GLUT_LEFT_BUTTON) { // was (button == GLUT_LEFT_BUTTON)
        LCD.touchState = true;
        LCD.touchX = x/ZOOM_SCALE; // re-enable these statements if you delete the passive mouse function
        LCD.touchY = y/ZOOM_SCALE;
    } if ((buttonState == GLUT_LEFT_BUTTON ) && (clickState == GLUT_UP)) {
        LCD.touchState = false;
        LCD.touchX = -1;
        LCD.touchY = -1;
    }
}

// keyboard callback
void Keyboard (unsigned char c, int x, int y) {
    switch (c) {                                // c is the key that is hit
        case 'a':
            leftKeyInp = true;
            break;
        case 'w':
            upKeyInp = true;
            break;
        case 'd':
            rightKeyInp = true;
            break;
        case 's':
            downKeyInp = true;
            break;
        case 27:                               // 'q' means quit
            exit (0);
            break;
        default:
            break;
    }
}

void KeyboardUp (unsigned char c, int x, int y) {
    if (!TIME_SCALE_ALTERATION_MODE) {
        switch (c) {                                // c is the key that is hit
            case 'a':
                leftKeyInp = false;
                break;
            case 'w':
                upKeyInp = false;
                break;
            case 'd':
                rightKeyInp = false;
                break;
            case 's':
                downKeyInp = false;
                break;
            default:
                break;
        }
    } else {
        float TIME_SCALE_INCREMENT_AMOUNT = 1.0;
        switch (c) {                                // c is the key that is hit
            case 'x':
                leftKeyInp = false;
                TIME_SCALE_FACTOR = 1.0 / 3.0;
                break;
            case 'a':
                leftKeyInp = false;
                TIME_SCALE_FACTOR = 1.0;
                break;
            case 'w':
                upKeyInp = false;
                TIME_SCALE_FACTOR = 10;
                break;
            case 'd':
                rightKeyInp = false;
                TIME_SCALE_FACTOR += TIME_SCALE_INCREMENT_AMOUNT;
                break;
            case 's':
                downKeyInp = false;
                TIME_SCALE_FACTOR -= TIME_SCALE_INCREMENT_AMOUNT;
                if (TIME_SCALE_FACTOR < 0) {
                    TIME_SCALE_FACTOR = 0;
                }
                break;
            default:
                break;
        }
        cout << "Time Scale: " << TIME_SCALE_FACTOR << endl;
    }
}

void Timer (int id) {
    mainLoop ();
    
    glutTimerFunc(TIMER_DELAY, Timer, 0);
}

int main (int argc, char** argv) {
    
    glutInit(&argc, argv);                      // OpenGL initializations
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);// double buffering and RGB // was GLUT_DOUBLE
    glutInitWindowSize(320*ZOOM_SCALE, 240*ZOOM_SCALE);               // create a 400x400 window
    glutInitWindowPosition(0, 0);               // ...in the upper left
    glutCreateWindow("Robot Environment");                  // create the window
    
    glutDisplayFunc(Display);                 // setup callbacks
    glutReshapeFunc(Reshape);
    glutMotionFunc(PassiveMouseV2);
    glutMouseFunc(Mouse);
    glutKeyboardFunc(Keyboard);
    glutKeyboardUpFunc(KeyboardUp);
    glutTimerFunc(TIMER_DELAY, Timer, 0);
    
    Init ();
    
    glutMainLoop ();                             // start it running
    
    return 0;                                   // ANSI C expects this
}






#endif
#if !IS_SIMULATION // if this is not the simulation, then this chunck of code is used. Otherwise, this code is ignored.
/*
 
 
 
 NON-simulation only stuff
 
 
 
 */







// include statements

#include <FEHIO.h>
#include <FEHRPS.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include "Simulation.hpp"
#include "stdio.h"


// priority function declarations
void Init ();
void mainLoop ();
void glutPostRedisplay (); // for simulation compatibility only; this function is never used in non-simulation version

// code control variables
float timeSinceLastCodeCall; // amount of time alotted since the mainLoop code was last called
#define TIMER_DELAY 0.00 // delay before the mainLoop is called; 0 means that the mainloop code is called as soon as it is able to
#define UPDATE_INTERVAL 0.5

// Don't add anything to this main method. If you do, it will mess up the simulation code's behavior.
// Only add stuff to the initialization method (Init) and the main looping method (mainLoop).
// It shouldn't be necessary to add to the main method itself.
int main (void) {
    RPS.InitializeTouchMenu ();
    
    // call the initilization method (is the first thing that gets called; only gets called once and is used for the purpose of initializing variables)
    Init ();
    
    while (true) {
        // call the mainLoop code only after a fixed time interval has passed; this is done to better mimic the similution code behavior
        if (TimeNow () - timeSinceLastCodeCall > TIMER_DELAY) {
            timeSinceLastCodeCall = TimeNow ();
            
            mainLoop ();
        }
    }
    
    return 0;
}






#endif
/*
 
 
 
 SHARED simulation & non-simulation stuff
 
 
 
 */






/************************************************/
/* Name: MC3                Date: 11/27/18      */
/* Instructor: RJF           10:20              */
/************************************************/

#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <string.h>
using namespace std;



/************************************************************************************************ DECLERATIONS ************************************************************************************************/


/************************************************************************ PRIORITY DECLERATIONS ************************************************************************/


void DrawBox (Box box);
void DrawBoxBorder (Box box);
void DrawEdge (Edge edge);
void DrawCircle (Circle circle);
void UpdateMotors ();
void UpdateBumps ();
void UpdateHardwareInput ();
void UpdateHardwareOutput ();
void PrintMotorPercents ();
int getRandomColor ();
void SetRandomColorIfStrobeMode ();


float SignOf (float num) {
    int result = 1;
    if (num < 0) {
        result = -1;
    }
    return result;
}

class PseudoRPS;
class PhysicsRPS;
class Course;
Vehicle subSimulationVehicle;

// bump identifiers
#define F1 2
#define B0 3
#define B1 4
#define D0 5
#define D1 0
#define F0 1

#define LOWERED_SERVO_ANGLE 75 // 65 // 85
#define RAISED_SERVO_ANGLE 170 // 170

// works only in simulation; if enabled, sets the vehicle's position to the position clicked on the screen
#define CLICK_MODE 1

Vector2 touch = Vector2 ();
bool touchState = false;
bool strobeMode = false;


/************************ HARDWARE VARIABLES ************************/

FEHMotor motor0 (FEHMotor::Motor0, 9.0); // the left motor
FEHMotor motor1 (FEHMotor::Motor1, 9.0); // the front motor
FEHMotor motor2 (FEHMotor::Motor2, 9.0); // the right motor

DigitalInputPin bump0 (FEHIO::P1_0); // D1
DigitalInputPin bump1 (FEHIO::P1_1); // F0
DigitalInputPin bump2 (FEHIO::P1_2); // F1
DigitalInputPin bump3 (FEHIO::P1_3); // B0
DigitalInputPin bump4 (FEHIO::P1_5); // B1
DigitalInputPin bump5 (FEHIO::P1_6); // D0

FEHServo servo0 (FEHServo::Servo0);
AnalogInputPin cds0 (FEHIO::P0_0);




/************************************************************************ DRAWING CLASS DECLERATIONS ************************************************************************/


class Course {
public:
    Course (Vector2 globalPos, float pixelScale) { // 116–117 px  ~  1 ft
        PIXEL_SCALE = pixelScale;
        INCHES_TO_PIXELS = 9.72222222222 * PIXEL_SCALE;
        pos = globalPos;
        
        CourseObjects objs = CourseObjects ();
        bounds = objs.bounds;
        
        bumb = objs.bumb;
        discoBall = objs.discoBall;
        obstacleBox = objs.obstacleBox;
        
        DDRPad = objs.DDRPad;
        DDRButtons = objs.DDRButtons;
        rightRamp = objs.rightRamp;
        
        foosball = objs.foosball;
        leverBank = objs.leverBank;
        tokenSlot = objs.tokenSlot;
        tokenBowl = objs.tokenBowl;
        
        leftRamp = objs.leftRamp;
        finalBank = objs.finalBank;
    }
    Course () {
        
    }
    Vector2 pos;
    float PIXEL_SCALE;
    float INCHES_TO_PIXELS;
    Box bounds;
    
    Box bumb; // 232x11
    Circle discoBall;
    Box obstacleBox;
    
    Box DDRPad; // 97x45
    Box DDRButtons; // 97x48
    Box rightRamp; // 115x240
    
    Box foosball; // 25x39 // 166x41
    Edge leverBank;
    Box tokenSlot; // 115x240
    Circle tokenBowl;
    
    Box leftRamp; // 115x87
    Edge finalBank;
    
    // draws the course
    void DrawCourse () {
        LCD.SetFontColor (GRAY);
        SetRandomColorIfStrobeMode ();
        DrawCourseObject (bounds);
        // LCD.SetFontColor (WHITE);
        // DrawOutlineOfBox (bounds);

        
        LCD.SetFontColor (DARKGRAY);
        SetRandomColorIfStrobeMode ();
        DrawCourseObject (leftRamp);
        DrawCourseObject (rightRamp);
        
        DrawCourseObject (DDRPad);
        
        DrawCourseObject (bumb);
        DrawCourseObject (discoBall);
        DrawCourseObject (tokenBowl);
        
        LCD.SetFontColor (LIGHTGRAY);
        SetRandomColorIfStrobeMode ();
        DrawCourseObject (DDRButtons);
        DrawCourseObject (foosball);
        
        DrawCourseObject (obstacleBox);
        DrawCourseObject (tokenSlot);
        
        DrawCourseObject (leverBank);
        DrawCourseObject (finalBank);
    }
    
    /************************ these functions (Draw Course Object) draw shapes where their coordinates are in PIXELS relative to the top left corner of the course rendering. They are also scaled by the PIXEL_SCALE constant. ************************/
    // draws a box object within the course
    void DrawCourseObject (Box obj) {
        // convert the box to global coordinates
        Box globalObj = Box ( ToGlobalPosition (obj.points[0]), obj.width * PIXEL_SCALE, obj.height * PIXEL_SCALE);
        // draw the global box through normal means
        DrawBox (globalObj);
    }
    // draws an outline of a box object within the course
    void DrawOutlineOfBox (Box obj) {
        // convert the box to global coordinates
        Box globalObj = Box ( ToGlobalPosition (obj.points[0]), obj.width * PIXEL_SCALE, obj.height * PIXEL_SCALE);
        // draw the global box through normal means
        DrawBoxBorder (globalObj);
    }
    // draws a circle object within the course
    void DrawCourseObject (Circle obj) {
        // convert the box to global coordinates
        Circle globalObj = Circle ( ToGlobalPosition (obj.points[0]), obj.radius * PIXEL_SCALE);
        // draw the global box through normal means
        DrawCircle (globalObj);
    }
    // draws an edge object within the course
    void DrawCourseObject (Edge obj) {
        // convert the box to global coordinates
        Edge globalObj = Edge ( ToGlobalPosition (obj.points[0]), ToGlobalPosition (obj.points[1]), 0);
        // draw the global box through normal means
        DrawEdge (globalObj);
    }
    
    /************************ these functions (Draw REAL Course Object) draw shapes where their coordinates are in INCHES relative to the top left corner of the course rendering. They are also indirectly scaled by the PIXEL_SCALE constant. ************************/
    // draws a box object within the course
    void DrawRealCourseObject (Box obj) {
        // convert the box to global coordinates
        Box globalObj = Box ( ToRealGlobalPosition (obj.points[0]), obj.width * INCHES_TO_PIXELS, obj.height * INCHES_TO_PIXELS);
        // draw the global box through normal means
        DrawBox (globalObj);
    }
    // draws a circle object within the course
    void DrawRealCourseObject (Circle obj) {
        // convert the box to global coordinates
        Circle globalObj = Circle ( ToRealGlobalPosition (obj.points[0]), obj.radius * INCHES_TO_PIXELS);
        // draw the global box through normal means
        DrawCircle (globalObj);
    }
    // draws an edge object within the course
    void DrawRealCourseObject (Edge obj) {
        // convert the box to global coordinates
        Edge globalObj = Edge ( ToRealGlobalPosition (obj.points[0]), ToRealGlobalPosition (obj.points[1]), 0);
        // draw the global box through normal means
        DrawEdge (globalObj);
    }
    void DrawRealCourseObject (Polygon poly) {
        // cout << "Length: " << poly.length << endl;
        for (int k = 0; k < poly.length-1; k++) {
            Edge edge = Edge (   Vector2 (poly.points[k].x, poly.points[k].y),  Vector2 (poly.points[k+1].x, poly.points[k+1].y),  0   );
            DrawRealCourseObject (edge);
        }
        Edge edge = Edge (   Vector2 (poly.points[poly.length-1].x, poly.points[poly.length-1].y),  Vector2 (poly.points[0].x, poly.points[0].y),  0   );
        DrawRealCourseObject (edge);
    }
    // draws a vector in real coords
    void PlotVector (Vector2 origin, Vector2 vect) {
        Vector2 endPoint = Vector2 (origin.x + vect.x, origin.y + vect.y);
        Edge vector = Edge (origin, endPoint, 0);
        DrawRealCourseObject (vector);
        
        if (IS_SIMULATION) {
            Circle startPoint = Circle (origin, 0.5);
            DrawRealCourseObject (startPoint);
        }
    }
    // draws text in real coords
    void DrawRealText (Vector2 point, const char *str) {
        Vector2 newPoint = ToRealGlobalPosition (point);
        LCD.WriteAt (str, newPoint.x - 7, -newPoint.y - 8);
    }
    
    // draws a vehicle object within the course
    void DrawRealVehicle (Vehicle vehicle) {
        // draw the vehicle's wheels; first convert them into proper coordinates
        for (int k = 0; k < vehicle.wheelsLength; k++) {
            Polygon local = vehicle.wheels[k].shape;
            // make wheel's shape points not relative to the vehicle by adding the vehicle's positition to each point
            Vector2 points[4];
            for (int q = 0; q < 4; q++) {
                points[q] = Vector2 (local.points[q].x + vehicle.pos.x, local.points[q].y + vehicle.pos.y);
            }
            Polygon global = Polygon (points, 4);
            DrawRealCourseObject (global);
        }
        
        // draw the actual vehicle; first convert it into proper coordinates
        Polygon local = vehicle.chassis;
        // make chassis's shape points not relative to the vehicle by adding the vehicle's positition to each point
        Vector2 points[6];
        for (int q = 0; q < 6; q++) {
            points[q] = Vector2 (local.points[q].x + vehicle.pos.x, local.points[q].y + vehicle.pos.y);
        }
        Polygon global = Polygon (points, 6);
        DrawRealCourseObject (global);
        
        // draw bump switches
        for (int k = 0; k < vehicle.bumpsLength; k++) {
            // make bump's point not relative to the vehicle by adding the vehicle's positition to each point
            Vector2 local = vehicle.bumps [k].pos;
            Vector2 global = Vector2 (local.x + vehicle.pos.x, local.y + vehicle.pos.y);
            
            // make the bump switches become longer when activated
            float scalar = 8.0;
            if (!vehicle.bumps [k].Value ()) {
                LCD.SetFontColor (GOLD);
                scalar *= 3.0;
            } else {
                LCD.SetFontColor (vehicle.color);
            }
            
            Vector2 scaledDir = Vector2 (vehicle.bumps [k].dir.x * scalar, vehicle.bumps [k].dir.y * scalar);
            
            // make the last bump switch blue in color
            if (k == 5) { // the last one
                LCD.SetFontColor (BLUE);
            }
            
            // draw bump switch labels
            /*
             Vector2 endPoint = Vector2 (global.x + scaledDir.x, global.y + scaledDir.y);
             switch (k) {
             case F0:
             DrawRealText (endPoint, "F");
             break;
             case D0:
             DrawRealText (endPoint, "D");
             break;
             case B0:
             DrawRealText (endPoint, "B");
             break;
             
             default:
             break;
             }
             */
            
            // draw bump switch direction indicators
            if (vehicle.isSimulated == 0) {
                // PlotVector (global, scaledDir);
            } else if (vehicle.isSimulated == 1) {
                local = vehicle.bumps [k].startPos;
                // global = Vector2 (local.x + vehicle.pos.x, local.y + vehicle.pos.y);
                global = local;
                PlotVector (global, Vector2 (vehicle.bumps [k].endPos.x - global.x, vehicle.bumps [k].endPos.y - global.y));
            }
        }
    }
    // draws real vehicle vectors (wheel direction, vehicle movement direction)
    void DrawVehicleVectors (Vehicle vehicle) {
        float scaler = 150.0;
        // draw vectors for each wheel
        for (int k = 0; k < vehicle.wheelsLength; k++) {
            Vector2 globalWheelPos = Vector2 (vehicle.wheels [k].pos.x + vehicle.pos.x, vehicle.wheels [k].pos.y + vehicle.pos.y);
            // Vector2 scaledDir = Vector2 (vehicle.wheels [k].dir.x * scaler, vehicle.wheels [k].dir.y * scaler);
            Vector2 scaledDir = Vector2 (vehicle.wheels [k].lastForceApplied.x * scaler, vehicle.wheels [k].lastForceApplied.y * scaler);
            PlotVector (globalWheelPos, scaledDir);
        }
        // draw the vehicle vector direction
        Vector2 scaledDir = Vector2 (vehicle.lastForceApplied.x * scaler, vehicle.lastForceApplied.y * scaler);
        PlotVector (vehicle.pos, scaledDir);
    }
    
    // converts a local position relative to the course to a global position
    Vector2 ToGlobalPosition (Vector2 local) {
        Vector2 global = Vector2 (local.x * PIXEL_SCALE + pos.x, local.y * PIXEL_SCALE + pos.y);
        return global;
    }
    // converts a local position relative to the course to a global position
    Vector2 ToRealGlobalPosition (Vector2 local) {
        Vector2 global = Vector2 (local.x * INCHES_TO_PIXELS + pos.x, local.y * INCHES_TO_PIXELS + pos.y);
        return global;
    }
    // converts a global position to a local position relative to the course
    Vector2 FromRealGlobalPosition (Vector2 global) {
        Vector2 local = Vector2 ((global.x - pos.x) / INCHES_TO_PIXELS, (global.y - pos.y) / INCHES_TO_PIXELS);
        return local;
    }
private:
};




/************************************************************************ NAVIGATION CLASS DECLERATIONS ************************************************************************/


/************************ MISC DECLARATIONS ************************/

class Navigator;
Course course;

class PseudoRPS {
public:
    PseudoRPS () {
        offset = Vector2 (0, 0);
        scale = Vector2 (1, 1);
        
        startAngle = 60.0; // 15.0
        lastValidCoordinate = Vector2 (0, 0);
        lastValidHeading = 0;
        safeMode = true;
        isValidSignal = false;
        isLikelyNewSignal = false;
        timeWhenLikelyUpdated = 0;
        
        // should change this to the start point
        firstTargetWaypoint = Vector2 (31.199, 12.800); // somewhere in the far corner by the ddr
        firstTargetWaypoint = Vector2 (8.800, 11.300); // at the start
    }
    float x;
    float y;
    float heading;
    
    Vector2 offset;
    Vector2 scale;
    
    Vector2 firstTargetWaypoint;
    
    float startAngle;
    
    float X () {
        if (!safeMode && isValid ()) {
            lastValidCoordinate.x = RPS.X ();
        }
        return ToCourseX (lastValidCoordinate.x);
    }
    float Y () {
        if (!safeMode && isValid ()) {
            lastValidCoordinate.y = RPS.Y ();
        }
        return ToCourseY (lastValidCoordinate.y);
    }
    // returns a Vector2 containing the last valid RPS coordinates
    Vector2 Pos () {
        if (!safeMode && isValid ()) {
            lastValidCoordinate = Vector2 (RPS.X (), RPS.Y ());
        }
        return Vector2 (lastValidCoordinate.x, lastValidCoordinate.y);
    }
    float Heading () {
        if (!safeMode && isValid ()) {
            lastValidHeading = Vector2::CapDegrees (RPS.Heading () + startAngle);
        }
        return lastValidHeading;
    }
    // returns whether current RPS coordinate is a valid coordinate
    bool isValid () {
        bool result = true;
        if (safeMode) {
            result = isValidSignal;
        } else if (RPS.X () < -0.5) {
            result = false;
        }
        return result;
    }
    float getTimeWhenLikelyUpdated () {
        return timeWhenLikelyUpdated;
    }
    // returns whether the RPS (probably) just updated or not
    float IsLikelyNewSignal () {
        return isLikelyNewSignal;
    }
    bool hasChangedSinceMovement (Vector2 startPosition) {
        bool result = false;
        float tolerance = 0.0001;
        if (!(abs (lastValidCoordinate.y - startPosition.y) < tolerance)) { // || isLikelyNewSignal  (to avoid it timing out ... add this if that happens)
            result = true;
        }
        return result;
    }
    
    float ToCourseX (float x) {
        float x0 = x * scale.x + offset.x;
        // float x0 = x;
        return x0;
    }
    float ToCourseY (float y) {
        float y0 = y * scale.y + offset.y;
        // float y0 = y;
        return y0;
    }
    
    void Calibrate (Vector2 realWorldWaypoint) { // float actual_x0, float actual_x1, float actual_y0, float actual_y1)
        float diff_x0 = firstTargetWaypoint.x - realWorldWaypoint.x;
        float diff_y0 = firstTargetWaypoint.y - realWorldWaypoint.y;
        offset.x = diff_x0;
        offset.y = diff_y0;
    }
    // hack RPS
    int HackRPS () {
        // int WaitForPacketDebug(int *packetsFound, int *packetsLost, int *lastFoundPacketTime);
        int result = 0;
        
        int packetsFound = 2;
        int packetsLost = 1;
        int lastFoundPacketTime = 50;
        
        int *packetsFoundPtr = &packetsFound;
        int *packetsLostPtr = &packetsLost;
        int *lastFoundPacketTimePtr = &lastFoundPacketTime;
        // RPS.WaitForPacketDebug (packetsFound, packetsLost, lastFoundPacketTime);
        
        result = *lastFoundPacketTimePtr;
        return result;
    }
    
    void EnableSafeMode () {
        safeMode = true;
    }
    void DisableSafeMode () {
        safeMode = false;
    }
    // updates the last valid coordinates (and heading)
    void Update () {
        float tempRPS_X = RPS.X ();
        float tempRPS_Y = RPS.Y ();
        float tempHeading = RPS.Heading ();
        float tempHeading2 = Vector2::CapDegrees (tempHeading + startAngle);
        isLikelyNewSignal = false;
        
        if (tempRPS_X > -0.5) {
            float tolerance = .0001; // .00001
            float degreeTolerance = .0001;
            float UPDATE_TIMEOUT_TIME = 1.5;
            
            if ( !(abs (lastValidCoordinate.x - tempRPS_X) < tolerance)  ||  !(abs (lastValidHeading - tempHeading2) < degreeTolerance)  ||  TimeNow () - timeWhenLikelyUpdated > UPDATE_TIMEOUT_TIME) { // prolly won't cause errors but this is an adjustment that could cause major changes in behavior
                isLikelyNewSignal = true;
                timeWhenLikelyUpdated = TimeNow ();
            }
            
            lastValidCoordinate = Vector2 (tempRPS_X, tempRPS_Y);
            lastValidHeading = Vector2::CapDegrees (tempHeading + startAngle);
            isValidSignal = true;
        } else {
            isValidSignal = false;
        }
    }
    
    Vector2 lastValidCoordinate;
    float lastValidHeading;
private:
    bool isValidSignal;
    float timeWhenLikelyUpdated;
    bool isLikelyNewSignal;
    bool safeMode; // when enabled, getting a coordinate requires the update function first (which gathers all the coordinate data all at once to prevent the possibility of getting different coordinates in between calls
};
PseudoRPS rps;


#if SUB_SIMULATION_ENABLED
class PhysicsRPS {
public:
    PhysicsRPS (PseudoRPS *rpsPtr, Vehicle *subSimulationVehiclePtr, Vehicle *inputVehiclePtr) {
        psRPS = rpsPtr;
        veh = subSimulationVehiclePtr;
        inputVeh = inputVehiclePtr;
        
        bool isSubSimulation = true;
        usesRPS = true;
        subSimulation = Simulation (veh, isSubSimulation);
        
        startAngle = 60;
        shouldFlip = true;
        heading = 315;
        justResumed = false;
    }
    PhysicsRPS () {
        
    }
    PseudoRPS *psRPS;
    Vehicle *veh; // sub-simulation vehicle
    Vehicle *inputVeh; // normal vehicle; contains hardware inputs
    Simulation subSimulation;

    float heading;
    float startAngle;
    
    
    // SETTING VARIABLES AND FUNCTIONS
    
    bool usesRPS; // will re-orient itself whenever there is an RPS update
    
    // sets whether the simulated vehicle will re-orient itself whenever there is an RPS update
    void SetRPSDependence (bool isEnabled) {
        usesRPS = isEnabled;
    }
    // set whether the simulated vehicle should collide or not
    void SetCollision (bool isEnabled) {
        subSimulation.collisionEnabled = isEnabled;
    }
    // set whether the simulated vehicle should have simulated bumpswitches
    void SetSimulatedBumps (bool isEnabled) {
        subSimulation.bumpsEnabled = isEnabled;
    }
    
    // CORE FUNCTIONS
    
    float X () {
        return rps.ToCourseX (veh->pos.x);
    }
    float Y () {
        // return rps.ToCourseY (veh->pos.y - 6 * 12);
        return rps.ToCourseY (veh->pos.y + 6 * 12);
    }
    float Heading () {
        // Vector2::CapDegrees (simulatedVehicle.dir.getAngle () - angleAtStart);
        // return heading;
        return heading + 0;
    }
    
    
    // MAIN
    
    void Resume () {
        subSimulation.timeSinceUpdate = TimeNow ();
        // heading = rps.Heading ();
        // justResumed = true;
        UpdateRPSOrientation ();
    }
    
    void UpdateRPSOrientation () {
        Vector2 updatedPosition = Vector2 (rps.lastValidCoordinate.x, rps.lastValidCoordinate.y - 6 * 12); // off by a bit because of calibration
        veh->SetPosition (updatedPosition);
        veh->SetRotation (Vector2::BetterCapDegrees (rps.Heading () - startAngle));
    }
    
    void Update () {
        float previousHeading = heading;
        Vector2 previousDir = Vector2 (veh->dir.x, veh->dir.y);
        
        veh->CopyHardwareValues (inputVeh);
        
        if (rps.IsLikelyNewSignal () || justResumed) {
            Vector2 updatedPosition = Vector2 (rps.lastValidCoordinate.x, rps.lastValidCoordinate.y - 6 * 12); // off by a bit because of calibration
            veh->SetPosition (updatedPosition);
            veh->SetRotation (Vector2::BetterCapDegrees (rps.Heading () - startAngle));
            justResumed = false;
        }
    
        subSimulation.Update ();
        
        heading = Vector2::CapDegrees (veh->dir.getEvenBetterAngle () + 30 - startAngle);
        
        heading = Vector2::CapDegrees (veh->dir.getBetterAngle () + 30 - startAngle);
        
        float rpsHeading = rps.Heading ();
        float headingDiff = Vector2::getDegreeDistance (heading, rpsHeading);
        float headingMinus180Diff = Vector2::getDegreeDistance (heading - 180, rpsHeading);

        if (abs (headingDiff) > abs (headingMinus180Diff)) {
            heading = Vector2::BetterCapDegrees (heading - 180);
        }
    }
private:
    bool shouldFlip;
    bool justResumed;
};
#else
class PhysicsRPS {
public:
    PhysicsRPS (PseudoRPS *rpsPtr, Vehicle *subSimulationVehiclePtr, Vehicle *inputVehiclePtr) {

    }
    PhysicsRPS () {
        
    }
    PseudoRPS *psRPS;
    Vehicle *veh; // sub-simulation vehicle
    Vehicle *inputVeh; // normal vehicle; contains hardware inputs
    
    // SETTING VARIABLES AND FUNCTIONS
    
    // sets whether the simulated vehicle will re-orient itself whenever there is an RPS update
    void SetRPSDependence (bool isEnabled) {

    }
    // set whether the simulated vehicle should collide or not
    void SetCollision (bool isEnabled) {

    }
    // set whether the simulated vehicle should have simulated bumpswitches
    void SetSimulatedBumps (bool isEnabled) {

    }
    
    // CORE FUNCTIONS
    
    float X () {
        return rps.X ();
    }
    float Y () {
        return rps.Y () * 0;
    }
    float Heading () {
        return rps.Heading () * 0;
    }
    
    
    // MAIN
    
    void Resume () {

    }
    
    void SetPositionToRPS () {

    }
    
    void Update () {

    }
private:
};
#endif

PhysicsRPS physics;




/************************ STATE DECLARATIONS ************************/

// this is the base class for the states used in the state machine object
class State {
public:
    State () {
        
    }
    
    // perform the state's actions
    virtual const bool Go (Navigator *navPtr) {
        return false;
    }
private:
};


// this object extends the State base class
class StateVoid : public State {
public:
    StateVoid (Navigator *navPtr, bool (Navigator::*funcPtr) ()) {
        nav = navPtr;
        functionPtr = funcPtr;
    }
    StateVoid () {
        
    }
    Navigator *nav;
    bool (Navigator::*functionPtr) (); // declares a pointer to a function
    
    // perform the state's actions; because this class is "virtual", it effectively overrides the State bases class's "Go" method when you make an object of this type (though the stuff you have to do with pointers to make this happen is a bit annoying)
    virtual const bool Go (Navigator *navPtr) {
        return (navPtr->*functionPtr) (); // calls the function in which functionPtr is pointing to
    }
private:
};


// this object extends the State base class
class StateVFFII : public State {
public:
    StateVFFII (Navigator *navPtr, bool (Navigator::*funcPtr) (Vector2, float, float, int, int), Vector2 vctr, float flt0, float flt1, int int0, int int1) {
        nav = navPtr;
        functionPtr = funcPtr;
        v0 = vctr;
        
        f0 = flt0;
        f1 = flt1;
        
        i0 = int0;
        i1 = int1;
    }
    StateVFFII () {
        
    }
    Navigator *nav;
    bool (Navigator::*functionPtr) (Vector2, float, float, int, int); // declares a pointer to a function
    
    Vector2 v0;
    
    float f0;
    float f1;
    
    int i0;
    int i1;
    
    // perform the state's actions; because this class is "virtual", it effectively overrides the State bases class's "Go" method when you make an object of this type (though the stuff you have to do with pointers to make this happen is a bit annoying)
    virtual const bool Go (Navigator *navPtr) {
        return (navPtr->*functionPtr) (v0, f0, f1, i0, i1); // calls the function in which functionPtr is pointing to
    }
private:
};


// this object extends the State base class
class StateVFFFI : public State {
public:
    StateVFFFI (Navigator *navPtr, bool (Navigator::*funcPtr) (Vector2, float, float, float, int), Vector2 vctr, float flt0, float flt1, float flt2, int int0) {
        nav = navPtr;
        functionPtr = funcPtr;
        v0 = vctr;
        f0 = flt0;
        f1 = flt1;
        f2 = flt2;
        i0 = int0;
    }
    StateVFFFI () {
        
    }
    Navigator *nav;
    bool (Navigator::*functionPtr) (Vector2, float, float, float, int); // declares a pointer to a function
    Vector2 v0;
    float f0;
    float f1;
    float f2;
    int i0;
    
    // perform the state's actions; because this class is "virtual", it effectively overrides the State bases class's "Go" method when you make an object of this type (though the stuff you have to do with pointers to make this happen is a bit annoying)
    virtual const bool Go (Navigator *navPtr) {
        return (navPtr->*functionPtr) (v0, f0, f1, f2, i0); // calls the function in which functionPtr is pointing to
    }
private:
};


// this object extends the State base class
class StateVFFI : public State {
public:
    StateVFFI (Navigator *navPtr, bool (Navigator::*funcPtr) (Vector2, float, float, int), Vector2 vctr, float flt0, float flt1, int int0) {
        nav = navPtr;
        functionPtr = funcPtr;
        v0 = vctr;
        
        f0 = flt0;
        f1 = flt1;
        
        i0 = int0;
    }
    StateVFFI () {
        
    }
    Navigator *nav;
    bool (Navigator::*functionPtr) (Vector2, float, float, int); // declares a pointer to a function
    
    Vector2 v0;
    
    float f0;
    float f1;
    
    int i0;
    
    // perform the state's actions; because this class is "virtual", it effectively overrides the State bases class's "Go" method when you make an object of this type (though the stuff you have to do with pointers to make this happen is a bit annoying)
    virtual const bool Go (Navigator *navPtr) {
        return (navPtr->*functionPtr) (v0, f0, f1, i0); // calls the function in which functionPtr is pointing to
    }
private:
};


// this object extends the State base class
class StateVFFF : public State {
public:
    StateVFFF (Navigator *navPtr, bool (Navigator::*funcPtr) (Vector2, float, float, float), Vector2 vctr, float flt0, float flt1, float flt2) {
        nav = navPtr;
        functionPtr = funcPtr;
        v0 = vctr;
        f0 = flt0;
        f1 = flt1;
        f2 = flt2;
    }
    StateVFFF () {
        
    }
    Navigator *nav;
    bool (Navigator::*functionPtr) (Vector2, float, float, float); // declares a pointer to a function
    Vector2 v0;
    float f0;
    float f1;
    float f2;
    
    // perform the state's actions; because this class is "virtual", it effectively overrides the State bases class's "Go" method when you make an object of this type (though the stuff you have to do with pointers to make this happen is a bit annoying)
    virtual const bool Go (Navigator *navPtr) {
        return (navPtr->*functionPtr) (v0, f0, f1, f2); // calls the function in which functionPtr is pointing to
    }
private:
};


// this object extends the State base class
class StateVFFFF : public State {
public:
    StateVFFFF (Navigator *navPtr, bool (Navigator::*funcPtr) (Vector2, float, float, float, float), Vector2 vctr, float flt0, float flt1, float flt2, float flt3) {
        nav = navPtr;
        functionPtr = funcPtr;
        v0 = vctr;
        f0 = flt0;
        f1 = flt1;
        f2 = flt2;
        f3 = flt3;
    }
    StateVFFFF () {
        
    }
    Navigator *nav;
    bool (Navigator::*functionPtr) (Vector2, float, float, float, float); // declares a pointer to a function
    Vector2 v0;
    float f0;
    float f1;
    float f2;
    float f3;
    
    // perform the state's actions; because this class is "virtual", it effectively overrides the State bases class's "Go" method when you make an object of this type (though the stuff you have to do with pointers to make this happen is a bit annoying)
    virtual const bool Go (Navigator *navPtr) {
        return (navPtr->*functionPtr) (v0, f0, f1, f2, f3); // calls the function in which functionPtr is pointing to
    }
private:
};


// this object extends the State base class
class StateVFF : public State {
public:
    StateVFF (Navigator *navPtr, bool (Navigator::*funcPtr) (Vector2, float, float), Vector2 vctr, float flt0, float flt1) {
        nav = navPtr;
        functionPtr = funcPtr;
        v0 = vctr;
        f0 = flt0;
        f1 = flt1;
    }
    StateVFF () {
        
    }
    Navigator *nav;
    bool (Navigator::*functionPtr) (Vector2, float, float); // declares a pointer to a function
    Vector2 v0;
    float f0;
    float f1;
    
    // perform the state's actions; because this class is "virtual", it effectively overrides the State bases class's "Go" method when you make an object of this type (though the stuff you have to do with pointers to make this happen is a bit annoying)
    virtual const bool Go (Navigator *navPtr) {
        return (navPtr->*functionPtr) (v0, f0, f1); // calls the function in which functionPtr is pointing to
    }
private:
};


// this object extends the State base class
class StateVFI : public State {
public:
    StateVFI (Navigator *navPtr, bool (Navigator::*funcPtr) (Vector2, float, int), Vector2 vctr, float flt0, int int0) {
        nav = navPtr;
        functionPtr = funcPtr;
        v0 = vctr;
        
        f0 = flt0;
        
        i0 = int0;
    }
    StateVFI () {
        
    }
    Navigator *nav;
    bool (Navigator::*functionPtr) (Vector2, float, int); // declares a pointer to a function
    
    Vector2 v0;
    
    float f0;
    
    int i0;
    
    // perform the state's actions; because this class is "virtual", it effectively overrides the State bases class's "Go" method when you make an object of this type (though the stuff you have to do with pointers to make this happen is a bit annoying)
    virtual const bool Go (Navigator *navPtr) {
        return (navPtr->*functionPtr) (v0, f0, i0); // calls the function in which functionPtr is pointing to
    }
private:
};


// this object extends the State base class
class StateFFI : public State {
public:
    StateFFI (Navigator *navPtr, bool (Navigator::*funcPtr) (float, float, int), float flt0, float flt1, int int0) {
        nav = navPtr;
        functionPtr = funcPtr;
        
        f0 = flt0;
        f1 = flt1;
        
        i0 = int0;
    }
    StateFFI () {
        
    }
    Navigator *nav;
    bool (Navigator::*functionPtr) (float, float, int); // declares a pointer to a function
    
    float f0;
    float f1;
    
    int i0;
    
    // perform the state's actions; because this class is "virtual", it effectively overrides the State bases class's "Go" method when you make an object of this type (though the stuff you have to do with pointers to make this happen is a bit annoying)
    virtual const bool Go (Navigator *navPtr) {
        return (navPtr->*functionPtr) (f0, f1, i0); // calls the function in which functionPtr is pointing to
    }
private:
};


// this object extends the State base class
class StateFF : public State {
public:
    StateFF (Navigator *navPtr, bool (Navigator::*funcPtr) (float, float), float flt0, float flt1) {
        nav = navPtr;
        functionPtr = funcPtr;
        
        f0 = flt0;
        f1 = flt1;
    }
    StateFF () {
        
    }
    Navigator *nav;
    bool (Navigator::*functionPtr) (float, float); // declares a pointer to a function
    
    float f0;
    float f1;
    
    // perform the state's actions; because this class is "virtual", it effectively overrides the State bases class's "Go" method when you make an object of this type (though the stuff you have to do with pointers to make this happen is a bit annoying)
    virtual const bool Go (Navigator *navPtr) {
        return (navPtr->*functionPtr) (f0, f1); // calls the function in which functionPtr is pointing to
    }
private:
};


// this object extends the State base class
class StateF : public State {
public:
    StateF (Navigator *navPtr, bool (Navigator::*funcPtr) (float), float flt0) {
        nav = navPtr;
        functionPtr = funcPtr;
        f0 = flt0;
    }
    StateF () {
        
    }
    Navigator *nav;
    bool (Navigator::*functionPtr) (float); // declares a pointer to a function
    float f0;
    
    // perform the state's actions; because this class is "virtual", it effectively overrides the State bases class's "Go" method when you make an object of this type (though the stuff you have to do with pointers to make this happen is a bit annoying)
    virtual const bool Go (Navigator *navPtr) {
        return (navPtr->*functionPtr) (f0); // calls the function in which functionPtr is pointing to
    }
private:
};


/************************ STATE MACHINE DECLARATION ************************/

class StateMachine {
public:
    StateMachine (float timeWhen, int presentState) { // create a state machine while also setting whether the SM has started and what its current state is
        currentState = presentState;
        length = 0;
        timeWhenStateChanged = timeWhen;
    }
    StateMachine (StateMachine *SM) { // copy constructor
        currentState = SM->currentState;
        length = SM->length;
        timeWhenStateChanged = SM->timeWhenStateChanged;
        // should techinically also deep copy the states array but I'm not messing with that
    }
    StateMachine () {
        currentState = 0;
        length = 0;
        timeWhenStateChanged = 0;
    }
    // vector<State*> states; // state data vector
    State *states [200]; // was 64 // 180
    int length; // the length of the state data vector
    int currentState;
    float timeWhenStateChanged;
    
    // adds an state/event/task
    void Add (State *newState) { // takes an State pointer as an argument
        // states.push_back (newState); // add the given state to the state data vector
        // states.resize (++length); // resize data vector // this is pretty inefficient but oh well
        states [length++] = newState;
    }
    // goes to the next state
    void Next () {
        // simultanesouly increment the current state of the state machine while checking if the current state was the final state
        if (++currentState >= length) {
            currentState = 0; // if the state previous state was the final state, then restart the state machine
        }
        timeWhenStateChanged = TimeNow ();
    }
    
    void Update (Navigator *navPtr) {
        bool stateFinished = states [currentState]->Go (navPtr);
        // if the end condition is met, then increment the current state and ~note the time in which the state ended
        if (stateFinished) {
            Next ();
        }
    }
    void Start () {
        currentState = 1;
        timeWhenStateChanged = TimeNow ();
    }
    void Reset () {
        currentState = 0;
    }
private:
};


/************************ NAVIGATOR DECLARATION ************************/

#define F_A_D 1.0 // used to accomodate variances in the Proteus's power for the foosball task by altering distances
#define F_A_P 0.8 // used to accomodate variances in the Proteus's power for the foosball task by altering power

#define L_A_D 0.8 // 1.0 // used to accomodate variances in the Proteus's power for the lever task by altering distances
// #define L_A_P 1.0 // used to accomodate variances in the Proteus's power for the lever task by altering power

class Navigator {
public:
    Navigator (Vehicle *vehicle) {
        veh = vehicle;
        SM = StateMachine ();
        
        initialized = false;
        
        
        // initialize constants
        
        STOP_DURATION = 0.5;
        XTRA_SHORT_STOP_DURATION = 0.1;
        SPEEDY_FACTOR = 1.3333333; // 1.0
        
        
        // initialize global variables
        
        substate = 0;
        
        timeWhenRotationStarted = 0;
        timeRequiredForRotation = 0;
        rotationDidNotJustStart = false;
        
        timeWhenMovementStarted = 0;
        timeRequiredForMovement = 0;
        movementDidNotJustStart = 0;
        
        leftTurnLimitReached = false;
        rightTurnLimitReached = false;
        
        minLightValue = 9999999;
        isBlue = false;
        moveCoordDistance = 0;
        moveCoordDirection = Vector2 (0, 0);
        rotateAngleRotation = 0;
        
        lastTouchCoordinate = Vector2 (0, 0);
        timeWhenValidRPSWasReceived = 0;
        currentIteration = 0;
        positionAtStartOfIteration = Vector2 (0, 0);
        
        
        // intialize physics variables
        
        alwaysRunSimulation = false;
        simulationRunning = alwaysRunSimulation;
        physics.SetSimulatedBumps (false);
        
        
        // initialize calibration variables
        
        DEFAULT_MAX_ANGULAR_VELOCITY = 135 * 2.0 * (1.0 / 1.15); // in degrees per second
        DEFAULT_MAX_VELOCITY = 17.8; // in inches per second
        
        max_angular_velocity = DEFAULT_MAX_ANGULAR_VELOCITY;
        max_velocity = DEFAULT_MAX_VELOCITY;
        
        DDRCoordinate = Vector2 (23.600 - 0.5, 15.000);
        
        // add states to the statemachine to perform the navigation procedure specified in the method below
        // PerformanceTestFour ();
    }
    Navigator () {
        
    }
    Vehicle *veh;
    StateMachine SM;
    
    bool initialized;
    int substate; // the state control variable used in any sub state machines used in the navigation procedure
    
    float timeWhenRotationStarted;
    float timeRequiredForRotation;
    bool rotationDidNotJustStart;
    
    float timeWhenMovementStarted;
    float timeRequiredForMovement;
    bool movementDidNotJustStart;
    
    float moveCoordDistance;
    Vector2 moveCoordDirection;
    float rotateAngleRotation;
    
    Vector2 lastTouchCoordinate;
    float timeWhenValidRPSWasReceived;
    int currentIteration; // used to keep track of how many RPS precision iterations have executed
    Vector2 positionAtStartOfIteration;
    
    // declare constants
    
    float STOP_DURATION; // variable for controlling the stop interval often used in navigation procedures
    float XTRA_SHORT_STOP_DURATION;
    float SPEEDY_FACTOR;
    
    
    // declare physics variables
    
    bool simulationRunning;
    bool alwaysRunSimulation;
    
    
    // declare calibration variables
    
    float DEFAULT_MAX_ANGULAR_VELOCITY;
    float DEFAULT_MAX_VELOCITY;
    float max_angular_velocity;
    float max_velocity;
    
    Vector2 calibrationStartPoint;
    float calibrationStartRotation;
    
    Vector2 DDRCoordinate; // coordinate used for navigating to DDR
    
    
    // PHYSICS FUNCTIONS
    
    // returns if the simulation is running
    bool IsSimulationRunning () {
        return simulationRunning;
    }
    void StartSimulation () {
        physics.Resume ();
        simulationRunning = true;
    }
    void EndSimulation () {
        simulationRunning = false || alwaysRunSimulation;
    }
    
    
    // CALIBRATION FUNCTIONS
    
    // sets the vehicle's max velocity, which is used to calculate distance in serval other functions
    void SetMaxVelocity (float newMaxVelocity) {
        max_velocity = newMaxVelocity;
    }
    // restores the vehicle's default max velocity
    void RestoreDefaultMaxVelocity () {
        max_velocity = DEFAULT_MAX_VELOCITY;
    }
    // sets the vehicle's max angular velocity, which is used to calculate rotation in serval other functions
    void SetMaxAngularVelocity (float newMaxAngularVelocity) {
        max_angular_velocity = newMaxAngularVelocity;
    }
    // restores the vehicle's default max angular velocity
    void RestoreDefaultMaxAngularVelocity () {
        max_angular_velocity = DEFAULT_MAX_ANGULAR_VELOCITY;
    }
    void SetDDRCoordinate (Vector2 coordinate) {
        DDRCoordinate = coordinate; // copies by reference
    }
    
    // calculates and returns the vehicles maximum velocity (which is the vehicle's hypothetical velocity at 100% power, assuming power and distance are linear)
    // assumes the vehicle goes straight and the RPS doesn't fuck up
    bool CalibrateDistance (Vector2 direction, float power, float duration) {
        bool result = false;
        // if (movementDidNotJustStart == false) {
        if (movementDidNotJustStart == false  &&  rps.isValid ()  &&  (rps.getTimeWhenLikelyUpdated () > SM.timeWhenStateChanged)) {
            
            // get the vehicle's start location via RPS
            calibrationStartPoint = Vector2 (rps.X (), rps.Y ());
            
            // move the vehicle for a duration at the given power
            veh->Move (direction, power);
            movementDidNotJustStart = true;
            timeWhenMovementStarted = TimeNow ();
        }
        if (movementDidNotJustStart) {
            if (TimeNow () - timeWhenMovementStarted > duration) {
                movementDidNotJustStart = false;
                
                // get the vehicle's stop location via RPS
                Vector2 endPoint = Vector2 (rps.X (), rps.Y ());
                // calculate distance
                float distanceTraveled = Vector2::Distance (calibrationStartPoint, endPoint);
                // calculate real duration
                float delta_time = rps.getTimeWhenLikelyUpdated () - timeWhenMovementStarted;
                // calculate velocity; velocity = distance / duration
                float velocity = distanceTraveled / delta_time;
                // calculate max velocity; power * factor = 100; factor = 100 / power; max_velocity = velocity * factor
                float factor = 100.0 / power;
                float calculatedMaxVelocity = velocity * factor;
                
                SetMaxVelocity (calculatedMaxVelocity);
                
                veh->Stop (); // remove rotation from vehicle
                result = true;
            }
        }
        
        return result;
    }
    
    // calculates and returns the vehicles maximum velocity (which is the vehicle's hypothetical velocity at 100% power, assuming power and distance are linear)
    // currently doesn't work with negative power; don't cross over 0 to 359
    bool CalibrateRotation (float power, float duration) {
        bool result = false;
        // if (rotationDidNotJustStart == false) {
        if (rotationDidNotJustStart == false  &&  rps.isValid ()  &&  (rps.getTimeWhenLikelyUpdated () > SM.timeWhenStateChanged)) {
            
            // get the vehicle's start location via RPS
            calibrationStartRotation = rps.Heading ();
            
            // move the vehicle for a duration at the given power
            veh->Turn (power);
            rotationDidNotJustStart = true;
            timeRequiredForRotation = TimeNow ();
        }
        if (rotationDidNotJustStart) {
            if (TimeNow () - timeRequiredForRotation > duration) {
                rotationDidNotJustStart = false;
                
                // get the vehicle's stop location via RPS
                float endRotation = rps.Heading ();
                // calculate distance
                float rotatedDegrees = endRotation - calibrationStartRotation;
                rotatedDegrees *= -1;
                // calculate real duration
                float delta_time = rps.getTimeWhenLikelyUpdated () - timeRequiredForRotation;
                // calculate velocity; velocity = distance / duration
                float angVelocity = rotatedDegrees / delta_time;
                // calculate max velocity; power * factor = 100; factor = 100 / power; max_velocity = velocity * factor
                float factor = 100.0 / power;
                float calculatedMaxAngularVelocity = angVelocity * factor;
                
                SetMaxAngularVelocity (calculatedMaxAngularVelocity);
                
                veh->Turn (-power); // "remove" the turn from the motors
                result = true;
            }
        }
        
        return result;
    }
    
    
    // turn a ceratin amount of degrees; must be called repeadetly from a loop; returns true once the turn is complete
    bool TurnDegrees (float degrees, float motorPercent) {
        // float ANG_VELOCITY_AT_50_POWER = 135; // in degrees per second // 150 // 120
        if (rotationDidNotJustStart == false) {
            veh->Turn (motorPercent);
            float angVel = max_angular_velocity * motorPercent / 100.0;
            timeRequiredForRotation = degrees / angVel;
            timeWhenRotationStarted = TimeNow ();
            rotationDidNotJustStart = true;
        }
        if (TimeNow () - timeWhenRotationStarted > timeRequiredForRotation) {
            rotationDidNotJustStart = false;
            veh->Turn (-motorPercent); // "remove" the turn from the motors
            return true;
        } else {
            return false;
        }
    }
    
    /*
     * Moves the vehicle to the given coordinate.
     * ()   Note the function assumes the RPS has returned a valid coordinate and angle.
     *
     * @return true when the action is complete, false otherwise.
     */
    bool RotateToGlobalAngle (float degrees, float power) {
        bool result = false;
        
        // check if this is the first call to the function and check if rps.isValid to ensure that the most recent coordinate is a valid rps coordinate
        if (rotationDidNotJustStart == false && rps.isValid ()) {
            float start = rps.Heading ();
            
            float deltaDegrees = degrees - start;
            deltaDegrees *= -1;
            
            float deltaDegreesMinus360 = deltaDegrees - 360;
            
            if (abs (deltaDegrees) > abs (deltaDegreesMinus360)) {
                deltaDegrees = deltaDegreesMinus360;
            }
            
            rotateAngleRotation = deltaDegrees;
            
            float signedPower = abs (power) * SignOf (rotateAngleRotation); // ensure that the power is in the correct direction // maybe this could be causing problems
            result = TurnDegrees (rotateAngleRotation, signedPower);
        } else if (rotationDidNotJustStart) {
            float signedPower = abs (power) * SignOf (rotateAngleRotation); // ensure that the power is in the correct direction
            result = TurnDegrees (rotateAngleRotation, signedPower);
        }
        
        return result;
    }
    
    bool PreciseRotateToGlobalAngle (float degrees, float power, int iterations) {
        bool result = false;
        bool isWithinTolerance = false;
        float tolerance = 3.0; // 0.2
        
        // check if this is the first call to the function and check if rps.isValid to ensure that the most recent coordinate is a valid rps coordinate
        if (rotationDidNotJustStart == false  &&  rps.isValid ()  &&  rps.getTimeWhenLikelyUpdated () > SM.timeWhenStateChanged) {
            float start = rps.Heading ();
            
            float deltaDegrees = degrees - start;
            deltaDegrees *= -1;
            
            float deltaDegreesMinus360 = deltaDegrees - 360;
            
            if (abs (deltaDegrees) > abs (deltaDegreesMinus360)) {
                deltaDegrees = deltaDegreesMinus360;
            }
            
            rotateAngleRotation = deltaDegrees;
            
            if (abs (rotateAngleRotation) < tolerance) {
                isWithinTolerance = true;
                currentIteration = 0;
                result = true;
            } else {
                
                float signedPower = abs (power) * SignOf (rotateAngleRotation); // ensure that the power is in the correct direction // maybe this could be causing problems
                result = TurnDegrees (rotateAngleRotation, signedPower);
                
            }
        } else if (rotationDidNotJustStart) {
            float signedPower = abs (power) * SignOf (rotateAngleRotation); // ensure that the power is in the correct direction
            result = TurnDegrees (rotateAngleRotation, signedPower);
        }
        
        if (result && !isWithinTolerance) {
            currentIteration++;
            if (currentIteration >= iterations) {
                currentIteration = 0;
            } else {
                result = false;
            }
        }
        
        return result;
    }
    
    // move a ceratin amount of inches; must be called repeadetly from a loop; returns true once the movement is complete
    bool MoveDistance (Vector2 direction, float motorPercent, float distance) {
        // around 17.8
        if (movementDidNotJustStart == false) {
            veh->Move (direction, motorPercent);
            float velocity = max_velocity * motorPercent / 100.0;
            timeRequiredForMovement = distance / velocity;
            timeWhenMovementStarted = TimeNow ();
            movementDidNotJustStart = true;
        }
        if (TimeNow () - timeWhenMovementStarted > timeRequiredForMovement) {
            movementDidNotJustStart = false;
            veh->Stop ();
            return true;
        } else {
            return false;
        }
    }
    
    /*
     * Moves the vehicle to the given coordinate; this function only uses RPS once at the very beginning.
     * // ()   Note the function assumes the RPS has returned a valid coordinate and angle.
     *
     * @return true when the action is complete, false otherwise.
     */
    bool MoveToCoordinate (Vector2 target, float power, float distanceError) {
        bool result = false;
        
        // check if this is the first call to the function and check if rps.isValid to ensure that the most recent coordinate is a valid rps coordinate
        if (movementDidNotJustStart == false && rps.isValid ()) {
            Vector2 start = Vector2 (rps.X (), rps.Y ());
            moveCoordDistance = Vector2::Distance (start, target) + distanceError;
            
            moveCoordDirection = Vector2 (target.x - start.x, target.y - start.y);
            moveCoordDirection = moveCoordDirection.getUnitVector ();
            moveCoordDirection = GetGlobalVector (moveCoordDirection);
            
            result = MoveDistance (moveCoordDirection, power, moveCoordDistance);
        } else if (movementDidNotJustStart) {
            result = MoveDistance (moveCoordDirection, power, moveCoordDistance);
        }
        
        return result;
    }
    
    /*
     * Moves the vehicle to the given coordinate; this function waits for a new valid RPS signal before executing.
     * ()   Will execute a single iteration if the given @iterations equals 0.
     */
    bool PreciseMoveToCoordinate (Vector2 target, float power, int iterations) {
        bool result = false;
        bool isWithinTolerance = false;
        float tolerance = 0.2; // 0.2
        
        float powerChangeCoefficient = 0.5;
        float alteredPower = power * pow (powerChangeCoefficient, currentIteration);
        
        // bool firstIterationCheck = currentIteration == 0  &&  rps.getTimeWhenLikelyUpdated () > SM.timeWhenStateChanged;
        
        // check if this is the first call to the function and check if rps.isValid to ensure that the most recent coordinate is a valid rps coordinate
        // if it's the first iteration, call the likelyUpdated function; if it's not, check if the RPS has atkeast updated since the first movement...
        if (movementDidNotJustStart == false  &&  rps.isValid ()  &&  (rps.getTimeWhenLikelyUpdated () > SM.timeWhenStateChanged)) {
            Vector2 start = Vector2 (rps.X (), rps.Y ());
            positionAtStartOfIteration = start;
            moveCoordDistance = Vector2::Distance (start, target);
            
            // if is within tolerance
            if (moveCoordDistance < tolerance) {
                isWithinTolerance = true;
                currentIteration = 0;
                result = true;
            } else {
                
                moveCoordDirection = Vector2 (target.x - start.x, target.y - start.y);
                moveCoordDirection = moveCoordDirection.getUnitVector ();
                moveCoordDirection = GetGlobalVector (moveCoordDirection);
                
                result = MoveDistance (moveCoordDirection, alteredPower, moveCoordDistance);
                
            }
        } else if (movementDidNotJustStart) {
            result = MoveDistance (moveCoordDirection, alteredPower, moveCoordDistance);
        }
        
        if (result && !isWithinTolerance) {
            currentIteration++;
            if (currentIteration >= iterations) {
                currentIteration = 0;
            } else {
                veh->Stop ();
                result = false;
                // manually update when the SM thinks the state changed
                SM.timeWhenStateChanged = TimeNow ();
            }
        }
        
        return result;
    }
    
    /*
     * Moves the vehicle to the given coordinate; this function adjusts the ropots movement whenever the RPS updates.
     */
    bool AssistedMoveToCoordinate (Vector2 target, float power, float distanceError) {
        bool result = false;
        
        // check if this is the first call to the function and check if rps.isValid to ensure that the most recent coordinate is a valid rps coordinate
        if (movementDidNotJustStart == false && rps.isValid ()) {
            Vector2 start = Vector2 (rps.X (), rps.Y ());
            moveCoordDistance = Vector2::Distance (start, target) + distanceError;
            
            moveCoordDirection = Vector2 (target.x - start.x, target.y - start.y);
            // moveCoordDirection = Vector2 (-3, .1);
            moveCoordDirection = moveCoordDirection.getUnitVector ();
            moveCoordDirection = GetGlobalVector (moveCoordDirection);
            
            result = MoveDistance (moveCoordDirection, power, moveCoordDistance);
        } else if (movementDidNotJustStart) {
            // if there is a valid RPS signal during the movement, recalculate the distance to travel so that it moves the target coordinate more accurately
            if (rps.isValid () && rps.getTimeWhenLikelyUpdated () > timeWhenMovementStarted) { // and is different than the last iteration
                // movementDidNotJustStart = false;
                
                Vector2 start = Vector2 (rps.X (), rps.Y ());
                moveCoordDistance = Vector2::Distance (start, target) + distanceError;
                
                moveCoordDirection = Vector2 (target.x - start.x, target.y - start.y);
                // moveCoordDirection = Vector2 (-3, .1);
                moveCoordDirection = moveCoordDirection.getUnitVector ();
                moveCoordDirection = GetGlobalVector (moveCoordDirection);
            }
            
            result = MoveDistance (moveCoordDirection, power, moveCoordDistance);
        }
        
        return result;
    }
    
    /************************ TASK FUNCTIONS ************************/
    
    /*
     * Stops the vehicle by setting the percent power of all of the wheels to zero. Waits @stopTime in seconds.
     * @return true when the action is complete, false otherwise.
     */
    bool StopVehicle (float stopTime) {
        veh->Stop ();
        // cout << "Stuff is happening." << endl;
        return (TimeNow() - SM.timeWhenStateChanged > stopTime);
    }
    
    /*
     * Rotates the vehicle clockwise with the given @power until the given angle has been reached.
     * (r1) Note that the rotation has not been completly calibrated, so it is currently not very accurate (though it should still be precise).
     * (r2) Note that power is added onto the current power of each motor, so the vehicle can theoretically move and rotate at the same time.
     * @degrees
     *      the target angle in degrees clockwise
     * @power
     *      the power dedicated to the rotation
     * @return true when the action is complete, false otherwise.
     */
    bool TurnCW (float degrees, float power) {
        // PrintMotorPercents ();
        return TurnDegrees (degrees, power);
    }
    
    /*
     * Rotates the vehicle counter clockwise with the given @power until the given angle has been reached.
     * (r1) Note that the rotation has not been completly calibrated, so it is currently not very accurate (though it should still be precise).
     * (r2) Note that power is added onto the current power of each motor, so the vehicle can theoretically move and rotate at the same time.
     *
     * @degrees
     *      the target angle in degrees counter clockwise
     * @power
     *      the power dedicated to the rotation
     *
     * @return true when the action is complete, false otherwise.
     */
    bool TurnCCW (float degrees, float power) {
        return TurnDegrees (-degrees, -power);
    }
    
    /*
     // Adds a rotation to the vehicle; immediately returns true; used for making the vehicle turn while moving
     bool AddRotationCW (float power) {
     veh->Turn (power);
     return true;
     }
     */
    
    /*
     * Rotates the vehicle clockwise with the given @power until the bump switch with index @bumpID is activated or until the max angle has been reached.
     * ()   Notes (r1) and (r2) still apply.
     * (r3) Note that this function does not "undo" the rotation unless the maximum angle angle is reached.
     *      So this function should generally be used only when a STOP function immediately follows it.
     *
     * @bumpID
     *      index of the bump switch that the function waits until is activated
     * @power
     *      the power dedicated to the rotation
     * @maxDegrees
     *      the maximum angle in degrees counter clockwise
     *
     * @return true when the action is complete, false otherwise.
     */
    bool TurnUntilBumpCW (float maxDegrees, float power, int bumpID) {
        bool result = !veh->bumps [bumpID].Value();
        if (result) {
            ResetRotateDegrees ();
        } else {
            result = TurnDegrees (maxDegrees, power);
        }
        return result;
    }
    
    /*
     * Rotates the vehicle counter clockwise with the given @power until the bump switch with index @bumpID is activated or until the max angle has been reached.
     * ()   Notes (r1) and (r2) still apply.
     * (r3) Note that this function does not "undo" the rotation unless the maximum angle angle is reached.
     *      So this function should generally be used only when a STOP function immediately follows it.
     * (*)  Note this function was not doing during performance test one.
     * (**) Note I changed the order of this function parameters, as well as the order of some other ones...
     *
     * @bumpID
     *      index of the bump switch that the function waits until is activated
     * @power
     *      the power dedicated to the rotation
     * @maxDegrees
     *      the maximum angle in degrees counter clockwise
     *
     * @return true when the action is complete, false otherwise.
     */
    bool TurnUntilBumpCCW (float maxDegrees, float power, int bumpID) {
        bool result = !veh->bumps [bumpID].Value();
        if (result) {
            ResetRotateDegrees ();
        } else {
            result = TurnDegrees (-maxDegrees, -power);
        }
        return result;
    }
    
    // resets the global variables associated with the rotate degrees function
    void ResetRotateDegrees () {
        rotationDidNotJustStart = false;
    }
    
    // resets the global variables associated with the move distance function
    void ResetMoveDistance () {
        movementDidNotJustStart = false;
        currentIteration = 0;
    }
    
    /*
     * Moves the vehicle in the given @direction with the given @power until the given @duration in seconds has passed.
     * @return true when the action is complete, false otherwise.
     */
    bool MoveDuration (Vector2 direction, float power, float duration) {
        veh->Move (direction, power);
        return (TimeNow() - SM.timeWhenStateChanged > duration);
    }
    
    /*
     * Moves the vehicle in the given global @direction with the given @power until the given @duration in seconds has passed.
     * @return true when the action is complete, false otherwise.
     */
    bool GlobalMoveDuration (Vector2 direction, float power, float duration) {
        // PrintMotorPercents ();
        veh->Move (GetGlobalVector (direction), power);
        return (TimeNow() - SM.timeWhenStateChanged > duration);
    }
    
    /*
     * Moves the vehicle in the given global @direction with the given @power until the vehicle has passed the given @xValue
     * or until the given @time out duration in seconds has passed.
     * @return true when the action is complete, false otherwise.
     */
    bool GlobalMoveUntilAboveX (Vector2 direction, float power, float timeOutDuration, float xValue) {
        bool result = rps.X () >= xValue || TimeNow() - SM.timeWhenStateChanged > timeOutDuration;
        // check if the action is complete; if it is, then mayhaps reset something
        if (result) {
            // reset
        } else {
            veh->Move (GetGlobalVector (direction), power);
        }
        return result;
    }
    // same as above method except with Y coordiante
    bool GlobalMoveUntilAboveY (Vector2 direction, float power, float timeOutDuration, float yValue) {
        bool result = rps.Y () >= yValue || TimeNow() - SM.timeWhenStateChanged > timeOutDuration;
        // check if the action is complete; if it is, then mayhaps reset something
        if (result) {
            // reset
        } else {
            veh->Move (GetGlobalVector (direction), power);
        }
        return result;
    }
    // same as above method except below X coordiante
    bool GlobalMoveUntilBelowX (Vector2 direction, float power, float timeOutDuration, float xValue) {
        bool result = rps.X () <= xValue || TimeNow() - SM.timeWhenStateChanged > timeOutDuration;
        // check if the action is complete; if it is, then mayhaps reset something
        if (result) {
            // reset
        } else {
            veh->Move (GetGlobalVector (direction), power);
        }
        return result;
    }
    // same as above method except with Y coordiante
    bool GlobalMoveUntilBelowY (Vector2 direction, float power, float timeOutDuration, float yValue) {
        bool result = rps.Y () <= yValue || TimeNow() - SM.timeWhenStateChanged > timeOutDuration;
        // check if the action is complete; if it is, then mayhaps reset something
        if (result) {
            // reset
        } else {
            veh->Move (GetGlobalVector (direction), power);
        }
        return result;
    }
    
    
    /*
     * Moves the vehicle in the given global @direction with the given @power while turning with the given @turnPower CCW until the given @duration in seconds has passed.
     * @return true when the action is complete, false otherwise.
     */
    bool GlobalMoveWhileTurningDuration (Vector2 direction, float power, float duration, float turnPower) {
        veh->Move (GetGlobalVector (direction), power);
        veh->Turn (-turnPower);
        return (TimeNow() - SM.timeWhenStateChanged > duration);
    }
    
    /*
     * Moves the vehicle in the given global @direction with the given @power while turning with the given @turnPower CCW
     * until the vehicle has passed the given @xValue or until the given @timeOutDuration in seconds has passed.
     * @return true when the action is complete, false otherwise.
     */
    bool GlobalMoveWhileTurningUntilAboveY (Vector2 direction, float power, float timeOutDuration, float turnPower, float yValue) {
        bool result = rps.Y () >= yValue || TimeNow() - SM.timeWhenStateChanged > timeOutDuration;
        // check if the action is complete; if it is, then mayhaps reset something
        if (result) {
            // reset
        } else {
            veh->Move (GetGlobalVector (direction), power);
            veh->Turn (-turnPower);
        }
        return result;
    }
    
    /*
     * Moves the vehicle in the given global @direction with the given @power while turning with the given @turnPower CCW
     * until the vehicle has passed the given @xValue or until the given @timeOutDuration in seconds has passed. Also
     * perform abort adjustment if the RPS signal is not valid.
     * @return true when the action is complete, false otherwise.
     */
    bool GlobalMoveWhileTurningWithAbortUntilAboveY (Vector2 direction, float power, float timeOutDuration, float turnPower, float yValue) {
        bool result = false;
        bool isValidSignal = rps.isValid ();
        float rotationErrorAmount = -10; // -30 // -10
        
        // check if the action is complete; if it is, then mayhaps reset something
        if (movementDidNotJustStart == false && isValidSignal) {
            movementDidNotJustStart = true;
            
            veh->Move (GetGlobalVector (direction), power);
            veh->Turn (-turnPower);
        } else if (!isValidSignal) {
            Vector2 inaccurateGlobalDirection = GetGlobalVector (direction);
            inaccurateGlobalDirection.Rotate (rotationErrorAmount); // the rotated amount should technically depend on the given turnPower but for our uses it won't ever matter
            
            veh->Move (inaccurateGlobalDirection, power + turnPower);
        } else {
            result = rps.Y () >= yValue || TimeNow() - SM.timeWhenStateChanged > timeOutDuration;
            
            if (result) {
                movementDidNotJustStart = false;
            } else {
                veh->Move (GetGlobalVector (direction), power);
                veh->Turn (-turnPower);
            }
        }
        
        return result;
    }
    
    /*
     * Moves the vehicle in the given global @direction with the given @power while turning with the given @turnPower CCW
     * until the vehicle has passed the given @xValue or until the given @timeOutDuration in seconds has passed. Also
     * perform abort adjustment if the RPS signal is not valid.
     * @return true when the action is complete, false otherwise.
     */
    bool PhysicsMoveWhileTurningUntilAboveY (Vector2 direction, float power, float timeOutDuration, float turnPower, float yValue) {
        bool result = false;
        bool isValidSignal = rps.isValid ();
        // float rotationErrorAmount = -10; // -30 // -10
        
        
        // check if the action is complete; if it is, then mayhaps reset something
        if (movementDidNotJustStart == false && isValidSignal) {
            movementDidNotJustStart = true;
            
            StartSimulation ();
            
            // veh->Move (GetGlobalPhysicsVector (direction), power);
            // veh->Turn (-turnPower);
        }
        if (movementDidNotJustStart) {
            if (simulationRunning) {
                physics.Update ();
            }
            result = physics.Y () >= yValue || TimeNow() - SM.timeWhenStateChanged > timeOutDuration;
            
            if (result) {
                movementDidNotJustStart = false;
                EndSimulation ();
            } else {
                veh->Move (GetGlobalPhysicsVector (direction), power);
                veh->Turn (-turnPower);
            }
        }
        
        return result;
    }
    // checks if the initial RPS signal is valid; doesn't end the simulation
    bool PhysicsMoveWhileTurningUntilAboveYStart (Vector2 direction, float power, float timeOutDuration, float turnPower, float yValue) {
        bool result = false;
        bool isValidSignal = rps.isValid ();

        // check if the action is complete
        if (movementDidNotJustStart == false && isValidSignal) {
            movementDidNotJustStart = true;
            
            StartSimulation ();
        }
        if (movementDidNotJustStart) {
            if (simulationRunning) {
                physics.Update ();
            }
            
            result = physics.Y () >= yValue || TimeNow() - SM.timeWhenStateChanged > timeOutDuration;
            
            if (result) {
                movementDidNotJustStart = false;
            } else {
                veh->Move (GetGlobalPhysicsVector (direction), power);
                veh->Turn (-turnPower);
            }
        }
        
        return result;
    }
    // doesn't check if the initial RPS signal is valid; ends the simulation
    bool PhysicsMoveWhileTurningUntilAboveYEnd (Vector2 direction, float power, float timeOutDuration, float turnPower, float yValue) {
        bool result = false;
        
        // check if the action is complete
        if (movementDidNotJustStart == false) {
            movementDidNotJustStart = true;
        }
        if (movementDidNotJustStart) {
            if (simulationRunning) {
                physics.Update ();
            }
            
            result = physics.Y () >= yValue || TimeNow() - SM.timeWhenStateChanged > timeOutDuration;
            
            if (result) {
                movementDidNotJustStart = false;
                EndSimulation ();
            } else {
                veh->Move (GetGlobalPhysicsVector (direction), power);
                veh->Turn (-turnPower);
            }
        }
        
        return result;
    }
    
    
    // converts global vector to local vector relative to the vehicle's orientation
    Vector2 GetGlobalPhysicsVector (Vector2 global) {
        Vector2 newGlobal = Vector2 (global.x, global.y);
        if (global.y == 0) {
            newGlobal.x *= -1;
        }
        float relativeDegrees = newGlobal.getCappedAngle () - physics.Heading () + 30;
        relativeDegrees = Vector2::CapDegrees (relativeDegrees);
        
        float lower = 90;
        float upper = 270;
        bool shouldFlip = false;
        
        if (relativeDegrees >= lower && relativeDegrees < upper) {
            shouldFlip = true;
        }
        
        Vector2 local = Vector2::DegreesToVector2 (relativeDegrees);
        local = local.getUnitVector ();
        
        if (shouldFlip) {
            local = Vector2 (-local.x, -local.y);
        }
        
        local.Rotate(-60);
        
        return local;
    }
    
    // converts global vector to local vector relative to the vehicle's orientation
    Vector2 GetGlobalVector (Vector2 global) {
        Vector2 newGlobal = Vector2 (global.x, global.y);
        if (global.y == 0) {
            newGlobal.x *= -1;
        }
        float relativeDegrees = newGlobal.getCappedAngle () - rps.Heading () + 30;
        relativeDegrees = Vector2::CapDegrees (relativeDegrees);
        
        float lower = 90;
        float upper = 270;
        bool shouldFlip = false;
        
        if (relativeDegrees >= lower && relativeDegrees < upper) {
            shouldFlip = true;
        }
        
        Vector2 local = Vector2::DegreesToVector2 (relativeDegrees);
        local = local.getUnitVector ();
        
        if (shouldFlip) {
            local = Vector2 (-local.x, -local.y);
        }
        
        local.Rotate(-60);
        
        return local;
    }

    //bool result = !veh->bumps [bumpID].Value();
    bool MoveWhileTurningDuration (Vector2 direction, float power, float duration, float turnPower) {
        veh->Move (direction, power);
        veh->Turn (-turnPower);
        return (TimeNow() - SM.timeWhenStateChanged > duration);
    }
    
    bool MoveWhileTurningUntilBumpDuration (Vector2 direction, float power, float duration, float turnPower, int bumpID) {
        bool result = !veh->bumps [bumpID].Value() || (TimeNow() - SM.timeWhenStateChanged > duration);
        
        veh->Move (direction, power);
        veh->Turn (-turnPower);
        
        return result;
    }
    
    float minLightValue;
    bool MoveToGetMinLightValue (Vector2 direction, float power, float duration) {
        bool result = false;
        
        veh->Move (direction, power);
        if (cds0.Value() < minLightValue) {
            minLightValue = cds0.Value();
        }
        
        result = TimeNow() - SM.timeWhenStateChanged > duration;
        if (result) {
            veh->Stop ();
        }
        
        return result;
    }
    
    // gathers up min light value if there is not already a valid min light value
    bool MoveToGetMinLightValueIfInvalidReading (Vector2 direction, float power, float duration) {
        bool result = false;
        
        if (!movementDidNotJustStart) {
            bool hasLightReading = veh->cds.isRedLight (minLightValue) || veh->cds.isRedLight (minLightValue);
            if (hasLightReading) {
                result = true;
            }
            movementDidNotJustStart = true;
        } else {
            result = MoveToGetMinLightValue (direction, power, duration);
        }
        
        if (result) {
            movementDidNotJustStart = false;
        }
        
        return result;
    }

    
    void ResetMin () {
        minLightValue = 9999999;
        isBlue = false;
    }
    
    bool MoveDurationBlueShiftUsingMin () {
        bool complete = false;
        
        if (veh->cds.isRedLight (minLightValue)) {
            // cout << "Do red." << endl;
            complete = true;
        } else if (veh->cds.isBlueLight (minLightValue) || isBlue) {
            // cout << "Do blue." << endl;
            isBlue = true;
            Vector2 blueDDRCoordinate = Vector2 (DDRCoordinate.x + 4.5, DDRCoordinate.y); // + 5.0
            complete = PreciseMoveToCoordinate (blueDDRCoordinate, 35, 1);
        } else {
            complete = true;
        }
        if (complete) {
            isBlue = false;
        }
        
        return complete;
    }
    
    bool isBlue;
    bool MoveDurationBlueShift() {
        bool complete = false;
        // bool isBlue=false;
        
        if (veh->cds.isRedLight ()) {
            complete = true;
        } else if (veh->cds.isBlueLight () || isBlue){
            isBlue=true;
            veh->Move (Vector.DE, 35);
            complete = (TimeNow () - SM.timeWhenStateChanged > .7);
        } else {
            complete = (TimeNow () - SM.timeWhenStateChanged > 2.0);// time-out error
        }
        if (complete) {
            isBlue = false;
        }
        
        return complete;
    }
    
    /* Maintains being flush with a wall while moving in a direction.
     * Note that bump0 and bump1 must correstpond to a 0 and 1 bump switch ID
     * in order for the function to know which way to turn. Also assumes that the
     * vehicle is already flush with the wall on the appropriate side. Note
     * that the direction should be going towards both the wall and the
     * desired direction in which the vehicle should go.
     *
     * @bump0
     *      bump switch identifier 0
     * @bump1
     *      bump switch identifier 1
     *
     * @return true when the action is complete, false otherwise.
     */
    bool leftTurnLimitReached;
    bool rightTurnLimitReached;
    bool MoveWhileFlushDuration (Vector2 direction, float power, float duration, int bump0, int bump1) {
        float maxDegrees = 30;
        float turnPower = power / 4.0;
        
        veh->Move (direction, power); // (note that the turn function adds a rotation to the vehicle without stopping it; only when the turn is complete does the function reset/undo the added rotation; by having this statement at the top, this function effectively resets the rotation then adds it again as necessary (which is useful because the rotation is not necessarily complete according to the turnDegree function's requirements))
        
        bool isTouchedB0 = !veh->bumps [bump0].Value();
        bool isTouchedB1 = !veh->bumps [bump1].Value();
        // if neither of the switches are touched, then don't do anything (the vehicle should move in the appropriate direction until a switch is touched)
        // if both of the switches are touched, then don't do anything
        // if the left switch is touched (B0), then rotate counter clockwise until the other switch is touched
        // if the right switch is touched (B1), then rotate clockwise until the other switch is touched
        if (isTouchedB0 && !isTouchedB1) {
            if (!leftTurnLimitReached) { // if the left turn limit has been reached, do not turn left anymore
                if (TurnDegrees (-maxDegrees, -turnPower)) {
                    leftTurnLimitReached = true;
                }
            }
        } else if (!isTouchedB0 && isTouchedB1){
            if (!rightTurnLimitReached) { // if the right turn limit has been reached, do not turn right anymore
                if (TurnDegrees (maxDegrees, turnPower)) {
                    rightTurnLimitReached = true;
                }
            }
        } else if (isTouchedB0 && isTouchedB1) { // if both are touching then reset the turn limit
            leftTurnLimitReached = false;
            rightTurnLimitReached = false;
        }
        
        bool isFinished = (TimeNow() - SM.timeWhenStateChanged > duration);
        if (isFinished) {
            leftTurnLimitReached = false;
            rightTurnLimitReached = false;
        }
        return isFinished;
    }
    
    /*
     * Moves the vehicle in the given @direction with the given @power until the bump switch with index @bumpID is activated.
     * @return true when the action is complete, false otherwise.
     */
    bool MoveUntilBump (Vector2 direction, float power, int bumpID) {
        veh->Move (direction, power);
        return !veh->bumps [bumpID].Value();
    }
    
    
    /*
     * Moves the vehicle in the given @direction with the given @power while the bump switch with index @bumpID is activated.
     * @return true when the action is complete, false otherwise.
     */
    bool MoveWhileBump (Vector2 direction, float power, int bumpID) {
        veh->Move (direction, power);
        return veh->bumps [bumpID].Value();
    }
    
    
    /*
     * Moves the vehicle in the given @direction with the given @power until the bump switch with index @bumpID is activated or until the @maxDuration is reached.
     * @return true when the action is complete, false otherwise.
     */
    bool MoveUntilBumpDuration (Vector2 direction, float power, float maxDuration, int bumpID) {
        veh->Move (direction, power);
        return !veh->bumps [bumpID].Value() || (TimeNow() - SM.timeWhenStateChanged > maxDuration);
    }
    
    
    
    
    /************************************************ ABORTION STATES ************************************************/
    
    
    bool MoveToCoordinateIfNoLightReading (Vector2 target, float power, int iterations) {
        bool result = true;
        bool hasLightReading = veh->cds.isRedLight (minLightValue) || veh->cds.isBlueLight (minLightValue);
        
        if (!hasLightReading) {
            result = PreciseMoveToCoordinate (target, power, iterations);
        }
        
        return result;
    }
    
    
    /*
     * Moves the vehicle in the given @direction with the given @power until the bump switch with index @bumpID is activated or until the @maxDuration is reached.
     * If it times, it performs an abort procedure to ensure it gets to the foosball wall
     * @return true when the action is complete, false otherwise.
     */
    bool MoveUntilBumpDurationToGetToFoosballWall (Vector2 direction, float power, float maxDuration, int bumpID) {
        bool result = false;
        
        switch (substate) {
            case 0: {
                if (TimeNow() - SM.timeWhenStateChanged > maxDuration) {
                    veh->Stop ();
                    substate++;
                    SM.timeWhenStateChanged = TimeNow ();
                } else {
                    veh->Move (direction, power);
                    result = !veh->bumps [bumpID].Value();
                }
                break;
            }
            case 1: {
                // move to try to get out of the deadzone
                Vector2 getOutOfDeadzoneDirection = Vector2 (direction.x, direction.y);
                getOutOfDeadzoneDirection.Rotate (-90);
                
                float GOODPower = 60;
                float GOODMaxDistance = 45;
                
                bool isOutOfDeadzone = PerformLocalAbortion (getOutOfDeadzoneDirection, GOODPower, GOODMaxDistance);
                
                if (isOutOfDeadzone) {
                    substate++;
                    SM.timeWhenStateChanged = TimeNow ();
                }
                break;
            }
                
            case 2: {
                // move to top of ramp
                // StateVFF    * get_centered_on_ramp = new StateVFF (this, & Navigator :: MoveToCoordinate, Vector2 (29, 20.0), 50,  0.0); // coordinate, power, distanceError // 30, 20.0
                bool isCenteredOnRamp = MoveToCoordinate (Vector2 (30, 52), 50, 0.0);
                
                if (isCenteredOnRamp) {
                    substate++;
                    SM.timeWhenStateChanged = TimeNow ();
                }
                break;
            }
                
            case 3: {
                // rotate until facing the right wall
                // StateFF     * align_to_global_angle_before_ramp = new StateFF (this, & Navigator :: RotateToGlobalAngle, -90, 50);
                bool isRotatedTowardsRamp = RotateToGlobalAngle (-90, 35.0);
                
                if (isRotatedTowardsRamp) {
                    substate++;
                    SM.timeWhenStateChanged = TimeNow ();
                }
                break;
            }
                
            case 4: {
                // move until bump
                
                veh->Move (Vector.F, 50);
                float maxDuration = 5.0;
                result = !veh->bumps [F0].Value() || (TimeNow() - SM.timeWhenStateChanged > maxDuration);
                
                break;
            }
                
            default:
                break;
        }
        
        // if action complete, reset global variables
        if (result) {
            substate = 0;
        }
        
        return result;
    }
    
    
    /*
     * Checks if there is an RPS signal; if there is not, then perform an abort procedure consisting of going in the given global direction (which uses the last valid RPS signal) until there is an RPS signal.
     * @return true when the action is complete, false otherwise.
     */
    bool PerformGlobalAbortion (Vector2 direction, float power, float maxDistance) {
        bool result = false;
        
        // check if the RPS signal is valid
        if (rps.isValid ()) {
            result = true; // may want to keep moving to make sure the robot stays in a valid RPS region
        } else {
            Vector2 localDirection = GetGlobalVector (direction);
            result = MoveDistance (localDirection, power, maxDistance);
        }
        
        return result;
    }
    
    
    /*
     * Checks if there is an RPS signal; if there is not, then perform an abort procedure consisting of going in the given local direction (which uses the last valid RPS signal) until there is an RPS signal.
     * @return true when the action is complete, false otherwise.
     */
    bool PerformLocalAbortion (Vector2 direction, float power, float maxDistance) {
        bool result = false;
        
        // check if the RPS signal is valid
        if (rps.isValid ()) {
            result = true; // may want to keep moving to make sure the robot stays in a valid RPS region
        } else {
            result = MoveDistance (direction, power, maxDistance);
        }
        
        return result;
    }
    
    
    /*
     * Set the servos angle to @degrees, and waits for @waitTime in seconds until performing the next action.
     * @return true when the action is complete, false otherwise.
     */
    bool SetServoAngle (float degrees, float waitTime) {
        veh->servo.SetDegree (degrees);
        return (TimeNow() - SM.timeWhenStateChanged > waitTime);
    }
    
    bool EnableStrobeMode () {
        strobeMode = true;
        return true;
    }
    bool DisableStrobeMode () {
        strobeMode = false;
        return true;
    }
    
    bool WaitForBlueLight () {
        return veh->cds.isBlueLight();
    }
    
    bool WaitForRedLight () {
        return veh->cds.isRedLight();
    }
    
    /*
     * Wait for the CDS cell to detect that a light is on, before performing the next action.
     * @return true when the action is complete, false otherwise.
     */
    bool WaitForLight () {
        return veh->cds.isLight();
    }
    
    /*
     * Wait for the CDS cell to detect that a light is on, before performing the next action.
     * @return true when the action is complete, false otherwise.
     */
    bool WaitForLightDuration (float timeOutDuration) {
        return veh->cds.isLight() || (TimeNow() - SM.timeWhenStateChanged > timeOutDuration);
    }
    
    // update the navigator; must be called repeatedly for everything to work properly
    void Update () {
        UpdateHardwareInput ();
        
        if (!simulationRunning) {
            if (IS_SIMULATION) {
                subSimulationVehicle.SetPosition (Vector2 (simulatedVehicle.pos.x, simulatedVehicle.pos.y));
                subSimulationVehicle.SetRotation (simulatedVehicle.dir.getAngle ());
            } else {
                physics.UpdateRPSOrientation ();
            }
        }
        
        if (initialized) {
            SM.Update (this);
        }
        
        UpdateHardwareOutput ();
    }
    
    void DefaultStatesTemplate () {
        /************************ initialize SAMPLE state objects ************************/
        
        // FYI the (& Navigator :: FunctionName) syntax is used to create a pointer to a function existing within a navigator object; the spaces are for style
        State       * do_nothing = new State ();
        StateVoid   * wait_for_Light = new StateVoid (this, & Navigator :: WaitForLight);
        
        StateVFF    * move_distance = new StateVFF (this, & Navigator :: MoveDistance, Vector.EF, 35, 6.0); // direction, power, distance
        StateVFF    * move_duration = new StateVFF (this, & Navigator :: MoveDuration, Vector.EF, 35, 1.0); // direction, power, duration
        StateVFI    * move_bump = new StateVFI (this, & Navigator :: MoveUntilBump, Vector.D, 35, D0); // direction, power, bumpID
        StateVFFI   * move_bump_duration = new StateVFFI (this, & Navigator :: MoveUntilBumpDuration, Vector.D, 35, 3.0, D1);  // direction, power, duration, bumpID
        
        StateFF     * turn_clockwise = new StateFF (this, & Navigator :: TurnCW, 45, 35); // degrees, power
        StateFFI    * turn_clockwise_bump = new StateFFI (this, & Navigator :: TurnUntilBumpCW, 45, 35, D0); // maxDegrees, power, bumpID
        StateVFFFI   * move_turn_bump_duration = new StateVFFFI (this, & Navigator :: MoveWhileTurningUntilBumpDuration, Vector.D, 35, 10.0, -5.0, D1); // direction, power, maxDuration, turnPower, bumpID
        
        StateVFFII  * move_while_flush = new StateVFFII (this, & Navigator :: MoveWhileFlushDuration, Vector.E, 35, 4.20, F0, F1); // direction, power, duration, bump0, bump1
        StateFF     * set_servo_angle = new StateFF (this, & Navigator :: SetServoAngle, 90, 1.0); // degrees, waitTime
        
        StateVFF    * global_move_duration = new StateVFF (this, & Navigator :: GlobalMoveDuration, Vector.up, 35,  3.0); // direction, power, duration
        StateVFFFF  * global_move_while_turning_until_above_Y = new StateVFFFF (this, & Navigator :: GlobalMoveWhileTurningUntilAboveY, Vector2 (3, 1).getUnitVector (), 50, 50, 10, 40); // direction, power, timeOutDuration, turnPower, yValue
        StateVFFF   * global_move_until_above_Y = new StateVFFF (this, & Navigator :: GlobalMoveUntilAboveY, Vector.up, 35, 15.0, 40); // direction, power, timeOutDuration, yValue
        StateVFFF   * global_move_while_turning = new StateVFFF (this, & Navigator :: GlobalMoveWhileTurningDuration, Vector.up, 35, 1.0, 30); // direction, power, duration, turnPower
        StateVFF    * move_to_coordinate = new StateVFF (this, & Navigator :: MoveToCoordinate, Vector2 (10.0, 10.0), 35,  0.0); // coordinate, power, distanceError
        StateFF     * rotate_to_global_angle = new StateFF (this, & Navigator :: RotateToGlobalAngle, 30, 50); // degrees, power
        
        StateVFF    * perform_global_abortion = new StateVFF (this, & Navigator :: PerformGlobalAbortion, Vector.down, 35,  8.0); // coordinate, power, distanceError
        
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
    }
    
    // perform all of the navigation procedures associated with state machine test
    void Tester () {
        /************************ initialize state objects ************************/
        
        State       * do_nothing = new State ();
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        StateFF     * turnALittle = new StateFF (this, & Navigator :: TurnCW, 45, 35); // degrees, power
        
        
        /************************ ADD REFERENCES of all of the state objects to the state machine ************************/
        
        // add the do nothing state, which should always be the first state (state [0])
        SM.Add (  do_nothing   );
        
        
        SM.Add (  stop   );
        SM.Add (  turnALittle   );
        SM.Add (  stop   );
        SM.Add (  stop   );
        
        
        // perform the final stop
        SM.Add (  stop   );
    }
    
    
    
    // initalize complete navigation procedures that indirectly add states to the statemachine by calling other methods
    /************************************************ FULL NAVIGATION PROCEDURES ************************************************/
    
    
    // adds states to the state machine associated with the final competition navigation procedure
    void FinalCompetition () {
        /*
         * Robot at DDR = 23.599, 15.000
         * Robot at token dispenser = 13.800, 42.800
         * Robot before going to lever = 4.199, 45.900
         */
        InitialState ();
        GetToDDR ();
        DoDDR ();
        
        GetUpRamp ();
        GetToToken ();
        DoToken ();
        
        GetToLever ();
        DoLever ();
        
        GetToFoosball ();
        DoFoosball ();
        
        GetDownRamp ();
        GoPressFinalButton ();
    }
    
    
    void PerformanceTestFour () {
        InitialState ();
        GetToDDR ();
        DoDDR ();
        GetUpRamp ();
        GetToFoosball ();
        DoFoosball ();
        GetDownRamp ();
        GoPressFinalButton ();
    }
    
    
    
    
    // initalize chunks of navigation procedures that directly add states to the statemachine
    /************************************************ NAVIGATION PROCEDURE CHUNKS ************************************************/
    
    
    /*
     * Initialize the initial states, where the robot waits for the GUI start button to be pressed, then waits for the start light to turn on.
     */
    void InitialState () {
        // *********** INITIALIZE state objects *********** //
        
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        State       * do_nothing = new State ();
        StateF      * wait_for_Light = new StateF (this, & Navigator :: WaitForLightDuration, 30.0);
        
        
        // *********** ADD REFERENCES of state objects to state machine *********** //
        
        // add the do nothing state, which should always be the first state (state [0])
        SM.Add (  do_nothing  );
        SM.Add (  wait_for_Light  );
    }
    
    
    /*
     * START: The robot is at its starting position.
     * END: The robot is on the light at the DDR pad.
     */
    void GetToDDR () {
        // *********** INITIALIZE state objects *********** //
        
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        StateF      * short_stop = new StateF (this, & Navigator :: StopVehicle, 0.1); // stopTime
        StateFF     * calibration_rotation = new StateFF (this, & Navigator :: CalibrateRotation, 50, 2.0); // 35, 3.0
        StateFFI    * global_align_to_DDR = new StateFFI (this, & Navigator :: PreciseRotateToGlobalAngle, -180, 50 * SPEEDY_FACTOR, 1);
        StateVFF    * initial_move = new StateVFF (this, & Navigator :: CalibrateDistance, Vector.FA, 25, 3.0); // direction, power, duration // 0.8 // 2.0
        
        Vector2 DDRSpot = DDRCoordinate;
        StateVFI    * move_to_DDR_light = new StateVFI (this, & Navigator :: PreciseMoveToCoordinate, DDRSpot, 50 * SPEEDY_FACTOR, 1);
        StateVFI    * move_to_DDR_light_2 = new StateVFI (this, & Navigator :: PreciseMoveToCoordinate, DDRSpot, 25 * SPEEDY_FACTOR, 2);

        
        // *********** ADD REFERENCES of state objects to state machine *********** //
        
        SM.Add (  initial_move  );
        SM.Add (  short_stop  );
        SM.Add (  stop  );
        SM.Add (  calibration_rotation  );
        SM.Add (  stop  );
        SM.Add (  global_align_to_DDR  );
        SM.Add (  stop  );
        SM.Add (  move_to_DDR_light  );
        SM.Add (  stop  );
        SM.Add (  move_to_DDR_light_2  );
        SM.Add (  stop  );
    }
    
    
    /*
     * START: The robot is on the light at the DDR pad.
     * END: The robot is at the DDR pad.
     */
    void DoDDR () {
        // *********** INITIALIZE state objects *********** //
        
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        StateF      * short_stop = new StateF (this, & Navigator :: StopVehicle, 0.25); // stopTime
        
        Vector2 DDRSpot = DDRCoordinate;
        StateVFF    * gather_up_min_lightValue = new StateVFF (this, & Navigator :: MoveToGetMinLightValue, Vector.DE, 35 * SPEEDY_FACTOR, 0.1); // direction, power, duration // MoveToGetMinLightValue
        StateVFI    * move_to_DDR_light_if_invalid_reading = new StateVFI (this, & Navigator :: MoveToCoordinateIfNoLightReading, DDRSpot, 25, 2); // coordinate, power, distanceError // 24, 15 // 22, 15 // 23.5, 14.5 // 23.600 - 0.5
        StateVFF    * retry_gathering_up_min_lightValue = new StateVFF (this, & Navigator :: MoveToGetMinLightValueIfInvalidReading, Vector.DE, 35, 0.1); // direction, power, duration // MoveToGetMinLightValue
        
        StateVoid   * blue_shift_using_min = new StateVoid (this, & Navigator :: MoveDurationBlueShiftUsingMin); //
        StateFFI    * global_align_to_DDR = new StateFFI (this, & Navigator :: PreciseRotateToGlobalAngle, -180, 50 / 4.0, 1); // 179 // (hyp. works) 179+45
        StateVFF    * move_button_F = new StateVFF (this, & Navigator :: MoveDuration, Vector.F, 35, 1.05); // direction, power, duration
        StateF      * hold_button = new StateF (this, & Navigator :: StopVehicle, 5.0); // stopTime
        
        
        // *********** ADD REFERENCES of state objects to state machine *********** //
        
        SM.Add (  global_align_to_DDR  );
        SM.Add (  stop  );
        
        SM.Add (  gather_up_min_lightValue  );
        SM.Add (  short_stop  );
        SM.Add (  move_to_DDR_light_if_invalid_reading  );
        SM.Add (  short_stop  );
        SM.Add (  retry_gathering_up_min_lightValue  );
        SM.Add (  short_stop  );
        
        SM.Add (  blue_shift_using_min  );
        SM.Add (  stop  );
        SM.Add (   move_button_F  );
        SM.Add (  hold_button  );
        SM.Add (  stop  );
    }
    
    
    /*
     * START: The robot is at the DDR pad.
     * END: The robot is centered at the top of the ramp.
     */
    void GetUpRamp () {
        // *********** INITIALIZE state objects *********** //
        
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        StateVFF    * get_centered_on_ramp = new StateVFF (this, & Navigator :: MoveToCoordinate, Vector2 (29, 20.0), 50 * SPEEDY_FACTOR,  0.0); // coordinate, power, distanceError // 30, 20.0
        StateVFFFF  * global_move_while_turning_to_get_up_most_of_ramp = new StateVFFFF (this, & Navigator :: PhysicsMoveWhileTurningUntilAboveYStart, Vector2 (0, 1.0).getUnitVector (), 80, 30.0, 0, 34.0+1.0); // direction, power, timeOutDuration, turnPower, yValue // 80, ..., 15 // 80 // -0.115, 1.0; 75, ..., -20
        StateVFFFF  * global_move_while_turning_to_get_up_ramp = new StateVFFFF (this, & Navigator :: PhysicsMoveWhileTurningUntilAboveYEnd, Vector2 (-0.115, 1.0).getUnitVector (), 65, 30.0, -20-10, 50-2); // direction, power, timeOutDuration, turnPower, yValue // 80, ..., 15 // 80 // -0.115, 1.0; 75, ..., -20 // =15
        StateFF     * align_to_global_angle_before_ramp = new StateFF (this, & Navigator :: RotateToGlobalAngle, -90, 50 * SPEEDY_FACTOR);
        StateFF     * slower_align_to_global_angle_before_ramp = new StateFF (this, & Navigator :: RotateToGlobalAngle, -90, 25 * SPEEDY_FACTOR);
        StateFF     * realign_to_global_angle_after_ramp = new StateFF (this, & Navigator :: RotateToGlobalAngle, -90, 50 * SPEEDY_FACTOR); // degrees, power // 270 + angleOffset + 30 // -90 // 270
        StateVFF    * get_centered_on_top_of_ramp_before_foosball = new StateVFF (this, & Navigator :: MoveToCoordinate, Vector2 (30, 52), 50,  0.0); // coordinate, power, distanceError
        StateVFF    * make_sure_not_in_deadzone = new StateVFF (this, & Navigator :: PerformGlobalAbortion, Vector.down, 35 * SPEEDY_FACTOR,  24.0); // coordinate, power, distanceError
        
        
        // *********** ADD REFERENCES of state objects to state machine *********** //
        
        SM.Add (  get_centered_on_ramp  ); // state 10
        SM.Add (  stop  );
        SM.Add (  align_to_global_angle_before_ramp  );
        SM.Add (  stop  );
        SM.Add (  slower_align_to_global_angle_before_ramp  );
        SM.Add (  stop  );
        
        // get up the ramp good
        SM.Add (  global_move_while_turning_to_get_up_most_of_ramp  );
        SM.Add (  global_move_while_turning_to_get_up_ramp  );
        SM.Add (  stop  );
        // make sure you are not in the deadzone
        SM.Add (  make_sure_not_in_deadzone  );
        SM.Add (  stop  );
        SM.Add (  get_centered_on_top_of_ramp_before_foosball  );
        SM.Add (  stop  );
        SM.Add (  make_sure_not_in_deadzone  );
        SM.Add (  realign_to_global_angle_after_ramp  );
        SM.Add (  stop  );
    }
    
    
    /*
     * START: The robot is centered at the top of the ramp.
     * END: The robot is at the token dispenser on the top platform.
     */
    void GetToToken () {
        // *********** INITIALIZE state objects *********** //
        
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        
        StateVFI    * move_left_to_token = new StateVFI (this, & Navigator :: PreciseMoveToCoordinate, Vector2 (20, 50-2), 50 * SPEEDY_FACTOR, 1); // coordinate, power, AssistedMoveToCoordinate
        
        Vector2 tokenDropSpot = Vector2 (13.800 + 1.2 + 0.5, 44.0);
        StateVFI    * move_dir_to_token = new StateVFI (this, & Navigator :: PreciseMoveToCoordinate, tokenDropSpot, 50 * SPEEDY_FACTOR, 1); // coordinate, power, distanceError
        StateVFI    * precise_move_dir_to_token = new StateVFI (this, & Navigator :: PreciseMoveToCoordinate, tokenDropSpot, 50 * 0.50 * SPEEDY_FACTOR, 3); // coordinate, power, distanceError
        
        
        // *********** ADD REFERENCES of state objects to state machine *********** //
        
        SM.Add (  move_left_to_token  );
        
        SM.Add (  move_dir_to_token  );
        SM.Add (  precise_move_dir_to_token  );
        SM.Add (  stop  );
        
    }
    
    
    /*
     * START: The robot is at the token dispenser on the top platform.
     * END: The robot is facing the token dispenser on the top platform (after dipensing the token).
     */
    void DoToken () {
        // *********** INITIALIZE state objects *********** //
        
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        StateFF     * lower_servo_for_token = new StateFF (this, & Navigator :: SetServoAngle, 35 + 70, 1.0); // degrees, waitTime
        StateFF     * raise_servo = new StateFF (this, & Navigator :: SetServoAngle, RAISED_SERVO_ANGLE, 1.0); // degrees, waitTime
        
        StateFFI    * global_rotate_to_token = new StateFFI (this, & Navigator :: PreciseRotateToGlobalAngle, 60, 35 * SPEEDY_FACTOR, 1); // degrees, power, iterations
        StateFFI    * global_rotate_to_token_with_less_power = new StateFFI (this, & Navigator :: PreciseRotateToGlobalAngle, 60, 12.5 * SPEEDY_FACTOR, 1); // degrees, power, iterations
        StateVFI    * touch_the_token = new StateVFI (this, & Navigator :: MoveUntilBump, Vector.D, 50 * SPEEDY_FACTOR, D0); // direction, power, bumpID
        
        
        // *********** ADD REFERENCES of state objects to state machine *********** //
        
        SM.Add (  global_rotate_to_token  );
        SM.Add (  stop  ); // move_to_token_again
        SM.Add (  global_rotate_to_token_with_less_power  );
        SM.Add (  stop  ); // move_to_token_again
        
        SM.Add (  touch_the_token  );
        SM.Add (  stop  );
        
        SM.Add (  lower_servo_for_token  );
        SM.Add (  raise_servo  );
    }
    
    void GetToLever () {
        // *********** INITIALIZE state objects *********** //
        
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        
        StateVFF    * make_sure_not_in_deadzone = new StateVFF (this, & Navigator :: PerformGlobalAbortion, Vector.down, 35, 12.0); // coordinate, power, F
        StateVFF    * move_after_token = new StateVFF (this, & Navigator :: MoveToCoordinate, Vector2 (7, 50-1), 50 * SPEEDY_FACTOR, 0.0); // coordinate, power, iterations
        StateFF     * rotate_to_face_left_wall = new StateFF (this, & Navigator :: RotateToGlobalAngle, -30, 35 * SPEEDY_FACTOR); // degrees, power // rotation angle may be incorrect
        StateFF     * rotate_to_face_left_wall_with_less_power = new StateFF (this, & Navigator :: RotateToGlobalAngle, -30, 35 / 2.0 * SPEEDY_FACTOR); // degrees, power // rotation angle may be incorrect
        StateVFI    * bump_into_left_wall = new StateVFI (this, & Navigator :: MoveUntilBump, Vector.D, 50 * SPEEDY_FACTOR, D0); // direction, power, bumpID
        StateVFFFI  * turn_until_bump_to_make_even = new StateVFFFI (this, & Navigator :: MoveWhileTurningUntilBumpDuration, Vector.D, 35 * SPEEDY_FACTOR, 10.0, 35, D1); // direction, power, maxDuration, turnPower, bumpID
        
        StateVFF    * move_upward_real_quick = new StateVFF (this, & Navigator :: MoveDistance, Vector.EF, 50 * SPEEDY_FACTOR, (5.0-2.0) * L_A_D); // direction, power, distance // 10 = about after gap // 8 // 7
        StateVFF    * move_back_from_left_wall = new StateVFF (this, & Navigator :: MoveDistance, Vector.Aa, 50 * SPEEDY_FACTOR, 1.0 * L_A_D); // direction, power, distance // 2.0
        
        StateFF     * rotate_to_lever = new StateFF (this, & Navigator :: TurnCW, 45, 50); // degrees, power // 15
        StateVFF    * move_right_before_bump = new StateVFF (this, & Navigator :: MoveDistance, Vector.E, 50 * SPEEDY_FACTOR, 5.0 * L_A_D); // direction, power
        StateVFI    * bump_into_lever_wall = new StateVFI (this, & Navigator :: MoveUntilBump, Vector.D, 50 * SPEEDY_FACTOR, D0); // direction, power, bumpID
        
        StateVFF    * move_back_from_lever_wall = new StateVFF (this, & Navigator :: MoveDistance, Vector.Aa, 50, 0.85 * L_A_D); // direction, power // 3.0 // 2.0 // 1.35

        StateVFFI   * move_right_before_lever_until_bump_1 = new StateVFFI (this, & Navigator :: MoveUntilBumpDuration, Vector.E, 50, 2.05, F0); // direction, power, maxDuration, bumpID // 30, 3.8 // 2.78
        StateVFFI   * move_right_before_lever_until_bump_2 = new StateVFFI (this, & Navigator :: MoveUntilBumpDuration, Vector.E, 30, 0.5, D1); // direction, power, maxDuration, bumpID // 0.6
        StateVFF    * move_left_before_lever = new StateVFF (this, & Navigator :: MoveDistance, Vector.BC, 35, 5.15 * L_A_D); // direction, power // 50 // 35 // 6.0 // 5.15 // 4.0?

        
        // *********** ADD REFERENCES of state objects to state machine *********** //
        
        SM.Add (  move_after_token  );
        SM.Add (  stop  );
        SM.Add (  make_sure_not_in_deadzone  );
        SM.Add (  rotate_to_face_left_wall  );
        SM.Add (  make_sure_not_in_deadzone  );
        SM.Add (  stop  );
        SM.Add (  rotate_to_face_left_wall_with_less_power  );
        SM.Add (  stop  );
        
        // 1. bump into left wall
        SM.Add (  bump_into_left_wall  );
        SM.Add (  stop  );
        SM.Add (  turn_until_bump_to_make_even  );
        SM.Add (  stop  );
        
        // 2. go up real quick for distance until around the gap
        SM.Add (  move_upward_real_quick  );
        SM.Add (  stop  );
        
        // 3. move back from wall
        SM.Add (  move_back_from_left_wall  );
        SM.Add (  stop  );
        
        // 4. rotate towards lever wall
        SM.Add (  rotate_to_lever  );
        SM.Add (  stop  );
        // move right
        SM.Add (  move_right_before_bump  );
        // 5. bump into the level wall
        SM.Add (  bump_into_lever_wall  );
        
        // move to get into that corner
        SM.Add (  move_right_before_lever_until_bump_1  );
        SM.Add (  move_right_before_lever_until_bump_2  );
        SM.Add (  stop  );
        
        // move left to get in front of lever
        SM.Add (  move_left_before_lever  );
        SM.Add (  stop  );
        
        // back up from the lever wall
        SM.Add (  move_back_from_lever_wall  );
        SM.Add (  stop  );
    }
    
    
    /*
     * START: The robot is at the lever.
     * END: The robot is facing the lever (after completing the lever task).
     */
    void DoLever () {
        // *********** INITIALIZE state objects *********** //
        
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        StateFF     * lower_servo_for_lever = new StateFF (this, & Navigator :: SetServoAngle, LOWERED_SERVO_ANGLE + 15, 1.0); // degrees, waitTime // 65.0 + 3.0 // + 30
        StateFF     * raise_servo = new StateFF (this, & Navigator :: SetServoAngle, RAISED_SERVO_ANGLE, 1.0); // degrees, waitTime
        StateVFF    * move_back_to_pull_lever = new StateVFF (this, & Navigator :: MoveDistance, Vector.Aa, 35, 1.3 * L_A_D); // direction, power // 3.0 // 2.0
        
        
        // *********** ADD REFERENCES of state objects to state machine *********** //
        
        // lower the servo
        SM.Add (  lower_servo_for_lever  );
        SM.Add (  move_back_to_pull_lever  );
        SM.Add (  stop  );
        SM.Add (  raise_servo  );
    }
    
    
    /*
     * START: The robot is facing the lever.
     * END: The robot is snuggly at the corner formed by the foosball box and the right wall.
     */
    void GetToFoosball () {
        // *********** INITIALIZE state objects *********** //
        
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        StateVFF    * move_right_after_lever = new StateVFF (this, & Navigator :: MoveDistance, Vector.EF, 50, 2.5); // direction, power // 0.15 (duration) // 4.0 ... 3.0 ... 2.85 // 2.5
        
        StateFF     * align_to_foosball = new StateFF (this, & Navigator :: TurnCW, 15, 35); // degrees, power // 15 // 25
        StateVFFI   * move_right_parralel_to_rod_until_bump = new StateVFFI (this, & Navigator :: MoveUntilBumpDurationToGetToFoosballWall, Vector.F, 50 * SPEEDY_FACTOR, 7.5, F0); // direction, power, maxDuration, bumpID
        StateVFI    * move_up_to_get_snug_with_foosball_box = new StateVFI (this, & Navigator :: MoveUntilBump, Vector.E, 35, D1); // direction, power // 0.16 // .25 // 0.20 // DE
        StateVFFFI  * rotate_until_bump_to_get_snug = new StateVFFFI (this, & Navigator :: MoveWhileTurningUntilBumpDuration, Vector.F, 35, 10.0, 35, F1); // direction, power, maxDuration, turnPower, bumpID

        
        // *********** ADD REFERENCES of state objects to state machine *********** //
        
        // move right relative to vehicle in order to avoid the disco ball
        SM.Add (  move_right_after_lever  );
        SM.Add (  stop  );
        // rotate clockwise some until the vehicle is parralel to the foosball rod  // (... or is facing the right wall)
        SM.Add (  align_to_foosball  );
        SM.Add (  stop  );
        // move rightwards until the rightmost bump switch touches the wall
        SM.Add (  move_right_parralel_to_rod_until_bump  );
        SM.Add (  stop  );
        SM.Add (  rotate_until_bump_to_get_snug  );
        SM.Add (  stop  );
        // do adjustments necessary so that the vehicle is snug against the wall
        SM.Add (  move_up_to_get_snug_with_foosball_box  );
        SM.Add (  stop  );
    }
    
    
    /*
     * START: The robot is snuggly at the corner formed by the foosball box and the right wall.
     * END: The robot is somewhere around the corner formed by the foosball box and the right wall (after completing the foosball task).
     */
    void DoFoosball () {
        // *********** INITIALIZE state objects *********** //
        // velocity conversion factor = 17.8

        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        StateFF     * raise_servo = new StateFF (this, & Navigator :: SetServoAngle, RAISED_SERVO_ANGLE, 1.0); // degrees, waitTime
        // StateFF     * lower_servo = new StateFF (this, & Navigator :: SetServoAngle, LOWERED_SERVO_ANGLE, 1.0); // degrees, waitTime
        StateFF     * lower_servo_for_foosball_drag = new StateFF (this, & Navigator :: SetServoAngle, LOWERED_SERVO_ANGLE, 1.0); // degrees, waitTime
        
        // move back a bit
        StateVFF    * move_back_from_foosball = new StateVFF (this, & Navigator :: MoveDistance, Vector.AB, 35 * F_A_P, 1.0 * F_A_D); // direction, power, duration
        // move left a little
        StateVFF    * move_to_align_to_foosball = new StateVFF (this, & Navigator :: MoveDistance, Vector.C, 35 * F_A_P, 4.0 * F_A_D); // 6.0
        // rotate a little to align to foosball
        StateFF     * turn_to_align_to_foosball = new StateFF (this, & Navigator :: TurnCW, 35 * F_A_D, 35 * F_A_P); // degrees, power // 30-5
        // move forward towards the foolsball disc to touch them
        StateVFF    * move_to_touch_foosball = new StateVFF (this, & Navigator :: MoveDistance, Vector.D, 35 * F_A_P, 1.2 * F_A_D); // direction, power // 0.16 // .25 // 0.20
        // drag foosball leftwards
        StateVFFF   * drag_foosball_leftwards = new StateVFFF (this, & Navigator :: MoveWhileTurningDuration, Vector.BC, 35 * F_A_P, 1.6 * F_A_D, 5 * F_A_P); // EF // was 2.0 // then 1.5 // 20 // -20 // 2.0 // 10
        
        // push the foosball discs again
        StateVFF    * move_right_before_push = new StateVFF (this, & Navigator :: MoveDuration, Vector.EF, 35 * F_A_P, 1.3175 * F_A_D); // direction, power // 0.16 // .25 // 0.20 // 1.0
        StateVFFF   * push_foosball_leftwards = new StateVFFF (this, & Navigator :: MoveWhileTurningDuration, Vector.BC, 35 * F_A_P, 1.5 * F_A_D, 2 * F_A_P); // 1.0-0.15 // 0.85 // 1.05
        StateVFF    * move_forward_before_push = new StateVFF (this, & Navigator :: MoveDistance, Vector.D, 35 * F_A_P, 0.6 * F_A_D); // direction, power // 0.16 // .25 // 0.20
        StateFF     * lower_servo_for_foosball_push = new StateFF (this, & Navigator :: SetServoAngle, LOWERED_SERVO_ANGLE - 5.0, 1.0); // degrees, waitTime
        
        // move back a bit
        StateVFF    * move_back_from_foosball_again = new StateVFF (this, & Navigator :: MoveDuration, Vector.Aa, 35 * F_A_P, 0.3 * F_A_D); // direction, power, duration
        // straighten
        StateFF     * straighten_after_foosball = new StateFF (this, & Navigator :: TurnCW, 1 * F_A_D, 35 * F_A_P); // degrees, power // 35
        StateVFFI   * move_right_after_foosball = new StateVFFI (this, & Navigator :: MoveUntilBumpDuration, Vector.EF, 35, 6.0, F0); // direction, power, maxDuration, bumpID

        
        // *********** ADD REFERENCES of state objects to state machine *********** //
        
        // move back a bit
        SM.Add (  move_back_from_foosball  );
        SM.Add (  stop  );
        // move left a little
        SM.Add (  move_to_align_to_foosball  );
        SM.Add (  stop  );
        // rotate
        SM.Add (  turn_to_align_to_foosball  );
        SM.Add (  stop  );
        // move forward towards the foolsball disc to touch them
        SM.Add (  move_to_touch_foosball  );
        SM.Add (  stop  );
        //move the servo arm ... lower_servo_for_foosball
        SM.Add (  lower_servo_for_foosball_drag  );
        SM.Add (  stop  );
        SM.Add (  drag_foosball_leftwards  );
        SM.Add (  stop  );
        
        // go back and drag (or push) the servo a second time
        SM.Add (  raise_servo  );
        SM.Add (  stop  );
        // move parrallel to the bar
        SM.Add (  move_right_before_push  );
        SM.Add (  stop  );
        // move forward a little bit before push
        SM.Add (  move_forward_before_push  );
        SM.Add (  stop  );
        // lower servo
        SM.Add (  lower_servo_for_foosball_push  );
        SM.Add (  stop  );
        // push again
        SM.Add (  push_foosball_leftwards  );
        SM.Add (  stop  );
        
        // leave foosball
        SM.Add (  raise_servo  );
        SM.Add (  move_back_from_foosball_again  );
        SM.Add (  stop  );
        SM.Add (  straighten_after_foosball  );
        SM.Add (  stop  );
        SM.Add (  move_right_after_foosball  );
        SM.Add (  stop  );
    }

    
    /*
     * START: The robot is snuggly at the corner formed by the foosball box and the right wall.
     * END: The robot is at the bottom of the ramp.
     */
    void GetDownRamp () {
        // *********** INITIALIZE state objects *********** //
        
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        StateVFF    * move_down_to_get_to_ramp = new StateVFF (this, & Navigator :: MoveDistance, Vector.Aa, 35 * SPEEDY_FACTOR, 16); // direction, power, distance // 18
        StateVFF    * make_sure_not_in_deadzone = new StateVFF (this, & Navigator :: PerformLocalAbortion, Vector.Aa, 35,  24.0); // coordinate, power, maxDistance
        StateVFFF   * global_move_to_get_down_ramp = new StateVFFF (this, & Navigator :: GlobalMoveUntilBelowY, Vector.down, 80, 15.0, 22); // direction, power, timeOutDuration, yValue // 65
        StateVFF    * get_centered_on_top_of_ramp = new StateVFF (this, & Navigator :: MoveToCoordinate, Vector2 (31, 52-1), 50 * SPEEDY_FACTOR,  0.0); // coordinate, power, distanceError // 30, 52 // 32, 52
        StateFF     * make_two_wheels_face_down_ramp = new StateFF (this, & Navigator :: RotateToGlobalAngle, -60 + 0 - 10, 50); // degrees, power
        
        
        // *********** ADD REFERENCES of state objects to state machine *********** //
        
        SM.Add (  move_down_to_get_to_ramp  ); // can prolly remove this state in the future
        SM.Add (  make_sure_not_in_deadzone  );
        SM.Add (  stop  );
        SM.Add (  get_centered_on_top_of_ramp  );
        SM.Add (  stop  );
        SM.Add (  make_sure_not_in_deadzone  );
        SM.Add (  stop  );
        SM.Add (  make_two_wheels_face_down_ramp  );
        SM.Add (  stop  );
        SM.Add (  global_move_to_get_down_ramp  );
        SM.Add (  stop  );
    }
    
    
    /*
     * START: The robot is at the bottom of the ramp.
     * END: The robot is pressed against the final button.
     */
    void GoPressFinalButton () {
        // *********** INITIALIZE state objects *********** //
        
        StateF      * stop = new StateF (this, & Navigator :: StopVehicle, STOP_DURATION); // stopTime
        StateVFF    * move_to_avoid_DDR = new StateVFF (this, & Navigator :: MoveToCoordinate, Vector2 (18, 14), 50 * SPEEDY_FACTOR,  0.0); // coordinate, power, distanceError // 6, 6
        StateVoid   * enable_strobe_mode = new StateVoid (this, & Navigator :: EnableStrobeMode);
        StateVoid   * disable_strobe_mode = new StateVoid (this, & Navigator :: DisableStrobeMode);
        StateVFF    * go_touch_that_button = new StateVFF (this, & Navigator :: MoveToCoordinate, Vector2 (7.3 - 3, 10.599 - 3), 50 * SPEEDY_FACTOR,  0.0); // coordinate, power, distanceError // 4, 6
        
        // the final stop
        StateF      * stop_at_the_end = new StateF (this, & Navigator :: StopVehicle, 0.01); // stopTime
        
        
        // *********** ADD REFERENCES of state objects to state machine *********** //

        // SM.Add (  enable_strobe_mode  );
        SM.Add (  move_to_avoid_DDR  );
        SM.Add (  go_touch_that_button  );
        // SM.Add (  disable_strobe_mode  );
        SM.Add (  stop  );
        SM.Add (  move_to_avoid_DDR  );
        SM.Add (  stop  );
        SM.Add (  go_touch_that_button  );
        SM.Add (  stop  );
        SM.Add (  move_to_avoid_DDR  );
        SM.Add (  stop  );
        SM.Add (  go_touch_that_button  );
        SM.Add (  stop  );
        SM.Add (  move_to_avoid_DDR  );
        SM.Add (  stop  );
        SM.Add (  go_touch_that_button  );
        SM.Add (  stop  );
        
        // perform the final stop
        SM.Add (  stop_at_the_end  );
    }
    
    
    // adds states to the statemachine; should only be called once
    void Initialize () {
        if (!initialized) {
            // apply the course calibration to the stored DDR coordinate
            DDRCoordinate = Vector2 (rps.ToCourseX (DDRCoordinate.x), rps.ToCourseY (DDRCoordinate.y));
            // add states to the statemachine that perform the navigation procedure specified in the below method
            FinalCompetition ();
            initialized = true;
        }
    }
    // resets the navigator object and stops the vehicle (resets state variables and that sort of thing)
    void Reset () {
        veh->Stop ();
        SM.Reset ();
        ResetRotateDegrees ();
        ResetMoveDistance ();
        ResetMin ();
    }
    // starts the navigation procedure; does this by setting state = 1; also initializes the statemachine (so the reset button secretely won't actually work, and it can't be calibrated after starting)
    void Start () {
        Initialize ();
        SM.Start ();
    }
    Vectors Vector;
private:
};




/************************************************************************ FUNCTION DECLERATIONS ************************************************************************/


/************************ MISC FUNCTIONS ************************/

Vector2 getCourseCalibration ();




/************************************************************************ GLOBAL VARIABLE DECLERATIONS ************************************************************************/


/************************ SETTINGS VARIABLES ************************/

// ...


/************************ MISCELLANEOUS VARIABLES ************************/

float timeSinceLastFrameUpdate;
char courseRegion;

Vehicle vehicle;
Simulation simulation;
Navigator navigator;

Vectors Vector;


/************************ GUI VARIABLES ************************/

// variables for button declaration
#define BUTT_WIDTH 100
#define BUTT_HEIGHT 30
#define BUTT_Y_POS 25
#define BUTT_X_POS 15
#define BUTT_X_MARGIN 15
#define BUTT_Y_MARGIN 10
Button butt;
Button cancelButt;
Button actionButt0;
Button actionButt1;
Button calibrateButt0;
Button calibrateButt1;

StateIndicator indicatorCDS;
StateIndicator indMisc0;
StateIndicator indMisc1;
StateIndicator indMotor0;
StateIndicator indMotor1;
StateIndicator indMotor2;
#define MOTOR_IND_Y_POS 165+20
#define MOTOR_IND_MARGIN 0
#define MOTOR_IND_MAX_WIDTH 174/2
#define MOTOR_IND_HEIGHT 11


/************************ Just some sample colors ************************/

#define ABYSS                   0x1a1a1a
#define BACKGROUNDCOLOR         0x1a1a1a     // 0x6495ed //- 0xCDDDF9 // 0x444243 // 0xAB7D5C // 0xFDF2E9
#define WALLCOLOR               0x1a1a1a     // 0x2f4f4f
#define WALLWALLCOLOR           0x8a715c     // 0x708090 // 0x418E88 // 0x9b704b // 0x8a715c
#define SHADOWCOLOR             0x000000
#define SHADOW_ALPHA            0.5          // between 0 and 1




/************************************************************************************************ MAIN FUNCTION ************************************************************************************************/


// do all the stuff that comes before the mainLoop
void Init () {
    LCD.Clear (BACKGROUNDCOLOR);
    // initialize some constants (that are only used in the initialization method)
    const double sqrt3 = pow (3.0, 0.5);
    srand (TimeNow ());
    
    // initialize buttons
    char charPtr [100];
    strcpy (charPtr, "START"); // use charPtr to set the text of the button
    // initialize the start button where its parameters are: a box containing the position of the top left corner of the button, the button's width, and the button's height; a pointer to the text contained within the button; and an integer containing the button's display type (what kind of button is drawn, which coded in the button class itself)
    butt = Button (  Box ( Vector2 (BUTT_X_POS, -BUTT_Y_POS), BUTT_WIDTH, BUTT_HEIGHT ), charPtr, 0  ); // butt = Button (  Box ( Vector2 (30, -60), BUTT_WIDTH, BUTT_HEIGHT ), charPtr, 0  );
    
    strcpy (charPtr, "RESET"); // use charPtr to set the text of the button
    // initialize the reset button where its parameters are the same as the one's mentioned above
    cancelButt = Button (  Box ( Vector2 (BUTT_X_POS, -BUTT_Y_POS - BUTT_HEIGHT*1 - BUTT_Y_MARGIN*1), BUTT_WIDTH, BUTT_HEIGHT ), charPtr, 0  );
    
    strcpy (charPtr, "DWN"); // use charPtr to set the text of the button
    // initialize the action buttons where their parameters are the same as the one's mentioned above
    actionButt0 = Button (  Box ( Vector2 (BUTT_X_POS, -BUTT_Y_POS - BUTT_HEIGHT*2 - BUTT_Y_MARGIN*2), 45, BUTT_HEIGHT ), charPtr, 0  ); // -150
    strcpy (charPtr, "UP");
    actionButt1 = Button (  Box ( Vector2 (BUTT_X_POS + 55, -BUTT_Y_POS - BUTT_HEIGHT*2 - BUTT_Y_MARGIN*2), 45, BUTT_HEIGHT ), charPtr, 0  );
    
    strcpy (charPtr, "Val");
    indicatorCDS = StateIndicator (  Box ( Vector2 (125, -BUTT_Y_POS), 65, BUTT_HEIGHT ), charPtr  ); // StateIndicator (  Box ( Vector2 (rightButtMargin, -15), 45, BUTT_HEIGHT ), charPtr  );
    indMisc0 = StateIndicator (  Box ( Vector2 (125, -BUTT_Y_POS - BUTT_HEIGHT*1 - BUTT_Y_MARGIN*1), 65, BUTT_HEIGHT ), charPtr  );
    indMisc1 = StateIndicator (  Box ( Vector2 (125, -BUTT_Y_POS - BUTT_HEIGHT*2 - BUTT_Y_MARGIN*2), 65, BUTT_HEIGHT ), charPtr  );
    
    // initialize the motor state indicators
    strcpy (charPtr, "");
    indMotor0 = StateIndicator (  Box ( Vector2 (BUTT_X_POS + MOTOR_IND_MAX_WIDTH, -MOTOR_IND_Y_POS), MOTOR_IND_MAX_WIDTH, MOTOR_IND_HEIGHT ), charPtr  );
    indMotor1 = StateIndicator (  Box ( Vector2 (BUTT_X_POS + MOTOR_IND_MAX_WIDTH, -MOTOR_IND_Y_POS - MOTOR_IND_MARGIN*1 - MOTOR_IND_HEIGHT*1), MOTOR_IND_MAX_WIDTH, MOTOR_IND_HEIGHT ), charPtr  );
    indMotor2 = StateIndicator (  Box ( Vector2 (BUTT_X_POS + MOTOR_IND_MAX_WIDTH, -MOTOR_IND_Y_POS - MOTOR_IND_MARGIN*2 - MOTOR_IND_HEIGHT*2), MOTOR_IND_MAX_WIDTH, MOTOR_IND_HEIGHT ), charPtr  );
    indMotor0.UpdateColors (LIGHTGRAY);
    indMotor1.UpdateColors (LIGHTGRAY);
    indMotor2.UpdateColors (LIGHTGRAY);
    
    strcpy (charPtr, "CAL"); // use charPtr to set the text of the button
    // initialize the reset button where its parameters are the same as the one's mentioned above
    calibrateButt0 = Button (  Box ( Vector2 (BUTT_X_POS, -MOTOR_IND_Y_POS - MOTOR_IND_MARGIN*3 - MOTOR_IND_HEIGHT*3 - BUTT_Y_MARGIN), 83, BUTT_HEIGHT ), charPtr, 0  );
    
    strcpy (charPtr, "DDR"); // use charPtr to set the text of the button
    calibrateButt1 = Button (  Box ( Vector2 (BUTT_X_POS + 92, -MOTOR_IND_Y_POS - MOTOR_IND_MARGIN*3 - MOTOR_IND_HEIGHT*3 - BUTT_Y_MARGIN), 83, BUTT_HEIGHT ), charPtr, 0  );

    // initialize the course object
    course = Course (Vector2 (200, -15), 0.3); // course = Course (Vector2 (150, -32), .25);
    
    // (note that all position numbers are in inches, and relative to the top left corner of the course unless specified otherwise)
    int reverse = -1; // this variable is used to accomidate for the direction of the wheels being reversed on the actual robot
    int wheelsLength = 3; // the amount of wheels on the robot; used to determine the length of the wheels array
    int bumpsLength = 6; // the amount of bump switches on the robot; used to determine the length of the bump switch array
    float radius = 1.2; // the radius of the vehicle's wheels; used for drawing purposes and simulation calculations
    float depth = 1.6; // the depth of the vehicle's wheels; used for drawing purposes
    float hexSide = 4.25; // the value of the longest side of the vehicle's chassis; also the largest "radius" of the hexagon
    float hexHeight = hexSide * sqrt3 / 2.0; // the smallest "radius" of the hexagon making up the vehicle
    float lowerHexRadius = (2.0*sqrt3 + 1.5*sqrt3) / 2.0; // (2*3^.5 + 1.5*3^.5) / 2.0 = 6.06217782649 / 2.0 ~= 3 // the "radius" of the lower hexagon's chassis; effectively just used to determine where the wheel's attach to the vehicles bottom
    Vector2 lowerDisp = Vector2 (0, 0.86602540378); // displacement between the lower chassis and the upper chassis; affects where the vehicle's wheels are positioned
    // initialize the wheel objects where their parameters are: the wheel's position relative to the position of the vehicle, the forward direction of the wheel, the wheel's radius, the wheel's depth
    Wheel leftWheel = Wheel ( Vector2 (lowerDisp.x - lowerHexRadius, lowerDisp.y - lowerHexRadius), Vector2 (1 * reverse,-sqrt3 * reverse), radius, depth );
    Wheel frontWheel = Wheel ( Vector2 (lowerDisp.x, lowerDisp.y + lowerHexRadius), Vector2 (-1 * reverse, 0), radius, depth );
    Wheel rightWheel = Wheel ( Vector2 (lowerDisp.x + lowerHexRadius, lowerDisp.y - lowerHexRadius), Vector2 (1 * reverse,sqrt3 * reverse), radius, depth );
    
    // initialize an array of the vehicle's wheels; necessary to create a vehicle object
    Wheel wheels[] = {   leftWheel, frontWheel, rightWheel  }; // 2.75 from number friendly center
    
    // initialize all the bump switches where their parameters are: its position relative to the vehicle, the direction it's facing
    BumpSwitch bump0 = BumpSwitch (Vector2 (-hexSide/2.0, -hexHeight), Vector2 (0, -1.0));
    BumpSwitch bump1 = BumpSwitch (Vector2 (-hexSide, 0), Vector2 (-sqrt3, 1));
    BumpSwitch bump2 = BumpSwitch (Vector2 (-hexSide/2.0, hexHeight), Vector2 (-sqrt3, 1));
    BumpSwitch bump3 = BumpSwitch (Vector2 (hexSide/2.0, hexHeight), Vector2 (sqrt3, 1));
    BumpSwitch bump4 = BumpSwitch (Vector2 (hexSide, 0), Vector2 (sqrt3, 1));
    BumpSwitch bump5 = BumpSwitch (Vector2 (hexSide/2.0, -hexHeight), Vector2 (0, -1.0));
    
    // initialize an array of the vehicle's bump switches; necessary to create a vehicle object
    BumpSwitch bumps[] = {   bump0, bump1, bump2, bump3, bump4, bump5   };
    
    // set up the servo; define its integer min and max numbers and set its start angle
    float servoStartDegrees = RAISED_SERVO_ANGLE;
    int servoMin = 500; // should be 694 // looks like this was never set properly during the performance test, ope
    int servoMax = 2500; // should be 1536 // looks like this was never set properly during the performance test, ope
    servo0.SetMin (servoMin);
    servo0.SetMax (servoMax);
    servo0.SetDegree (servoStartDegrees);
    // create a servor object where its parameters are: it's position relative to the vehicle, it's direction (which currently doesn't matter), it's starting angle
    Servo servo = Servo (Vector2 (0, 0.5), Vector2 (0, 1.0), servoStartDegrees);
    
    // create a CDS cell object where its parameters are: it's position relative to the vehicle, it's direction (which currently doesn't matter)
    CDS cds = CDS (Vector2 (0, 1.0), Vector2 (0, 1.0));
    
    // initalize the vehicle object
    Vector2 centerOfMass = Vector2 (0,0); // the vehicle's center of mass (relative to its geometric origin); currently has no affect on anything
    Vector2 startPosition = Vector2 (9,-63); // the vehicle's start position in inches relative to the top left corner of the course
    Vector2 startDirection = Vector2 (0,1); // the vehicle's start direction; somewhere along the way, these numbers got messed up and some work arounds had to be added to make the vehicle's orientation and direction match the actual robot; its best not to mess with the orientation and direction values...
    // create the shape of the vehicle's chassis
    Vector2 chPoints[6] = {Vector2 (-hexSide/2.0, hexHeight),  Vector2 (hexSide/2.0, hexHeight),  Vector2 (hexSide, 0),  Vector2 (hexSide/2.0, -hexHeight),  Vector2 (-hexSide/2.0, -hexHeight),  Vector2 (-hexSide, 0)};
    Polygon chassis = Polygon (chPoints, 6);
    // create the vehicle object where its parameters are: its start position, start direction, center of mass, its only servo object, its only cds cell object, an array of its wheel, the length of that wheel array, an array of its bump switches, the length of that bump switch array, the radius of the lower portion of its chassis or more specifically the radius of the circle in which its wheels are tangential to (which is used in simulation calculations)
    vehicle = Vehicle (startPosition, startDirection, centerOfMass, chassis, servo, cds, wheels, wheelsLength, bumps, bumpsLength, lowerHexRadius);
    // do stuff to fix the vehicles rotation to make it better match the orientation of the real robot
    vehicle.AddRotation (255); // 180-45+120
    
    if (IS_SIMULATION) {
        // create a vehicle object that shows where the vehicle would by according to simulated values; this should exist in the Xcode simulation
        bool isSubSimulation = false;
        simulatedVehicle = Vehicle (vehicle);
        simulation = Simulation (&simulatedVehicle, isSubSimulation);
        simulatedVehicle.color = BLACK;
        simulatedVehicle.isSimulated = true;
    }
    // initialize vehicle for subsimulation
    subSimulationVehicle = Vehicle (vehicle);
    subSimulationVehicle.color = WHITE;
    subSimulationVehicle.isSimulated = 2;
    
    vehicle.color = MAGENTA; // set the color in which the vehicle will appear as on the course rendering
    
    navigator = Navigator (&vehicle); // initalize the navigator object (which will be used to control vehicle navigation procedures)
    // do stuff to fix the vehicles rotation to make it better match the orientation of the real robot
    float alignAmount = 60;
    navigator.Vector.AlignVehicleVectors (alignAmount);
    Vector.AlignVehicleVectors (alignAmount);
    
    courseRegion = RPS.CurrentRegionLetter ();
    rps = PseudoRPS (); // initialize pseudo RPS object (which is essentially RPS but can be calibrated)
    
    Vector2 realWorldPoint = Vector2 (8.800, 11.300); // (8.800, 11.300);
    
    if (IS_SIMULATION) {
        realWorldPoint = Vector2 (9.0, 9.0);
        navigator.SetDDRCoordinate(Vector2 (24.343, 12.343));
    } else {
        // realWorldPoint = getCourseCalibration ();
    }
    
    rps.Calibrate (realWorldPoint);
    
    // setup subsimulation
    physics = PhysicsRPS (&rps, &subSimulationVehicle, &vehicle);
    
    // initialize the navigation procedure for the state machine (important that this happens after the vehicle vectors are aligned)
    // navigator.Initialize ();
    
    timeSinceLastFrameUpdate = TimeNow (); // initialize the time since the last screen/frame update
}


Vector2 getCourseCalibration () {
    Vector2 courseDefault = Vector2 (31.199, 12.800);
    Vector2 courseA = courseDefault;
    Vector2 courseB = Vector2 (31.199, 12.800); // one use for target coordinate for calibration
    Vector2 courseC = courseDefault;
    Vector2 courseD = Vector2 (30.900, 12.900);
    Vector2 courseE = courseDefault;
    Vector2 courseF = courseDefault;
    Vector2 courseG = courseDefault;
    Vector2 courseH = courseDefault;
    
    Vector2 courses [8] = {courseA, courseB, courseC, courseD, courseE, courseF, courseG, courseH};
    
    int courseID = RPS.CurrentRegion ();
    Vector2 calibrationCoordinateUsed = courseDefault;
    
    if (courseID >= 0  &&  courseID < 8) {
        calibrationCoordinateUsed = courses [courseID];
    }
    
    return calibrationCoordinateUsed;
}


// draws everything; all drawing functions should go here
// It's important to put all LCD functions here. It's critical for the simulation to work properly and
// is good coding practice for graphical computing, especially when you have limited computing capactity
// like you do on the Proteus.
void DrawEverything () {
    LCD.Clear (BACKGROUNDCOLOR); // clear the screen (for animation purposes)
    char charPtr [100];
    
    
    // draw the course
    course.DrawCourse ();
    
    // set the font color to the vehicle's color
    LCD.SetFontColor (vehicle.color); // ORCHID
    SetRandomColorIfStrobeMode ();
    // draw the vehicle on the course
    course.DrawRealVehicle (vehicle);
    
    
    if (true || navigator.IsSimulationRunning ()) { // SUB_SIMULATION_ENABLED
        // set the font color to the subsimulated vehicle's color
        LCD.SetFontColor (subSimulationVehicle.color);
        
        SetRandomColorIfStrobeMode ();
        
        // draw the subsimulation vehicle on the course
        course.DrawRealVehicle (subSimulationVehicle);
    }
    // if it is the simulation, draw the simulated vehicle
    if (IS_SIMULATION) {
        // set the font color to the simulated vehicle's color
        LCD.SetFontColor (simulatedVehicle.color);
        
        SetRandomColorIfStrobeMode ();
        
        // draw the subsimulation vehicle on the course
        course.DrawRealVehicle (simulatedVehicle);
        // set the font color to red
        LCD.SetFontColor (RED);
        // draw vectors indicating the forces applied to the vehicle onto the simulated vehicle
        course.DrawVehicleVectors (simulatedVehicle);
    }
    
    // draw CDS indicator
    int temp0 = 0;
    int temp1 = 0;
    
    float minLightValue = navigator.minLightValue;
    
    if (minLightValue < 66666) {
        if (vehicle.cds.isRedLight (minLightValue)) {
            indicatorCDS.UpdateColors (INDIANRED);
        } else if (vehicle.cds.isBlueLight (minLightValue)) {
            indicatorCDS.UpdateColors (LIGHTBLUE);
        } else {
            indicatorCDS.UpdateColors (GRAY);
        }
        temp0 = minLightValue * 1000;
    } else {
        indicatorCDS.UpdateColors (BACKGROUNDCOLOR);
        temp0 = vehicle.cds.Value () * 1000;
    }
    
    sprintf (charPtr, "%d", temp0);
    indicatorCDS.UpdateText (charPtr);
    indicatorCDS.Draw ();
    
    // draw misc indicators
    int temp2 = rps.Heading ();
    sprintf (charPtr, "%i", temp2);
    // sprintf (charPtr, "%i", navigator.SM.currentState);
    indMisc0.UpdateText (charPtr);
    
    if (rps.isValid ()) {
        indMisc0.UpdateColors (ABYSS);
        indMisc1.UpdateColors (ABYSS);
    } else {
        indMisc0.UpdateColors (CRIMSON);
        indMisc1.UpdateColors (CRIMSON);
    }
    temp0 = rps.X ();
    temp1 = rps.Y ();
    sprintf (charPtr, "%i, %i", temp0, temp1);
    indMisc1.UpdateText (charPtr);
    
    indMisc0.Draw ();
    indMisc1.Draw ();
    
    // draw motor indicators
    indMotor0.box.width = MOTOR_IND_MAX_WIDTH * (vehicle.wheels [2].activePercent / 100.0); // 0
    indMotor1.box.width = MOTOR_IND_MAX_WIDTH * (vehicle.wheels [0].activePercent / 100.0); // 1
    indMotor2.box.width = MOTOR_IND_MAX_WIDTH * (vehicle.wheels [1].activePercent / 100.0); // 2
    indMotor0.Draw ();
    indMotor1.Draw ();
    indMotor2.Draw ();
    LCD.SetFontColor (WHITE);
    DrawBoxBorder (  Box ( Vector2 (BUTT_X_POS, -MOTOR_IND_Y_POS), MOTOR_IND_MAX_WIDTH*2 + 1, MOTOR_IND_MARGIN*2 + MOTOR_IND_HEIGHT*3 )  );
    
    // draw buttons
    butt.DrawButton();
    cancelButt.DrawButton();
    actionButt0.DrawButton();
    actionButt1.DrawButton();
    calibrateButt0.DrawButton();
    calibrateButt1.DrawButton();
}


// do all of the stuff for the main game loop that gets repeated
void mainLoop () {
    touchState = LCD.Touch (&touch.x, &touch.y); // get the user's touch input coordinates, and store whether or not the user is touching the screen in the variable touchState
    touch = Vector2 (touch.x, -touch.y); // standardize the touch coordinates so they're effectively in the fourth quadrant
    
    // if this is the simulation and click mode is enable and the user touched the screen
    if (IS_SIMULATION) {
        if (CLICK_MODE == 1 && touchState) {
            // and the touch input is passed the start of the course rendering
            if (touch.x > course.pos.x) {
                // then set he vehicle's position to the touch coordinate
                simulatedVehicle.SetPosition ( course.FromRealGlobalPosition (touch) );
            }
        }
        // update the simulated vehicle every code iteration; will adjust the simulated vehicle's position and angle
        simulation.Update ();
    }
    
    if (actionButt0.IsBeingPressed (touch)) { // checks if the button was pressed and updates the state of the button
        vehicle.servo.SetDegree (LOWERED_SERVO_ANGLE); // sets the servo's angle to LOWERED_SERVO_ANGLE degrees
    }
    if (actionButt1.IsBeingPressed (touch)) { // checks if the button was pressed and updates the state of the button
        vehicle.servo.SetDegree (RAISED_SERVO_ANGLE); // sets the servo's angle to RAISED_SERVO_ANGLE degrees
    }
    
    if (cancelButt.IsBeingPressed (touch)) { // checks if the button was pressed and updates the state of the button
        navigator.Reset (); // resets the navigation procedure
    }
    if (butt.IsBeingPressed (touch)) { // checks if the button was pressed and updates the state of the button
        navigator.Start (); // manually starts the navigation procedure
    }
    if (calibrateButt0.IsBeingPressed (touch)) { // checks if the button was pressed and updates the state of the button
        rps.Calibrate (   Vector2 (RPS.X (), RPS.Y ())   ); // calibrate the pseudo rps using the vehicle's current RPS position
    }
    if (calibrateButt1.IsBeingPressed (touch)) { // checks if the button was pressed and updates the state of the button
        navigator.SetDDRCoordinate (   Vector2 (RPS.X (), RPS.Y ())   ); // set the coordinate in which the vehicle will navigate to in order to do the DDR
    }
    
    vehicle.SetPosition (Vector2 (rps.X (), rps.Y () - 6.0 * 12)); // set the position in which the vehicle appears on the screen
    vehicle.SetRotation (rps.Heading () + 240.0 - 60.0); // set the rotation of the vehicle that will appear on the screen
    navigator.Update (); // perform the entire navigation procedure

    // update the LCD screen only after a fixed time interval has passed
    if (TimeNow () - timeSinceLastFrameUpdate > UPDATE_INTERVAL) {
        timeSinceLastFrameUpdate = TimeNow ();
        
        if (IS_SIMULATION) {
            // basically just calls the DrawEverything function, which draws everything and updates the screen
            glutPostRedisplay (); // SIMULATION code chunk
        } else {
            DrawEverything (); // NON-simulation code chunk
        }
    }
    
}


// draws a box with the given box object
void DrawBox (Box box) {
    LCD.FillRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y), box.width, box.height);
}
// draws a box's border with the given box object
void DrawBoxBorder (Box box) {
    LCD.DrawRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y), box.width, box.height);
}

// draws given edge
void DrawEdge (Edge edge) {
    LCD.DrawLine (  (int)(edge.points[0].x), -(int)(edge.points[0].y), (int)(edge.points[1].x), -(int)(edge.points[1].y)  );
}

// draws given cricle
void DrawCircle (Circle circle) {
    LCD.FillCircle (  (int)(circle.points[0].x), -(int)(circle.points[0].y), circle.radius  );
}

// draws given polygon
void DrawPolygon (Polygon poly) {
    for (int k = 0; k < poly.length-1; k++) {
        LCD.DrawLine (  (int)(poly.points[k].x), -(int)(poly.points[k].y), (int)(poly.points[k+1].x), -(int)(poly.points[k+1].y)  );
    }
    LCD.DrawLine (  (int)(poly.points[poly.length-1].x), -(int)(poly.points[poly.length-1].y), (int)(poly.points[0].x), -(int)(poly.points[0].y)  ); // polygon.length - 1 ?
}


// updates the cds cell value that is stored in the vehicle object to the actual cds cell reading
void UpdateCDS () {
    vehicle.cds.value = cds0.Value ();
}
// updates the output servo angle to the servo angle value stored in the vehicle object
void UpdateServos () {
    servo0.SetDegree (vehicle.servo.degrees);
}
// updates the output motor percent values to the motor percent values stored in the vehicle object
void UpdateMotors () {
    motor0.SetPercent (vehicle.wheels[0].activePercent);
    motor1.SetPercent (vehicle.wheels[1].activePercent);
    motor2.SetPercent (vehicle.wheels[2].activePercent);
}
// updates the bump switch values that are stored in the vehicle object to the actual bump switch readings
void UpdateBumps () {
    vehicle.bumps [0].value = bump0.Value ();
    vehicle.bumps [1].value = bump1.Value ();
    vehicle.bumps [2].value = bump2.Value ();
    vehicle.bumps [3].value = bump3.Value ();
    vehicle.bumps [4].value = bump4.Value ();
    vehicle.bumps [5].value = bump5.Value ();
}
// updates all of the hardware input stored in the vehicle object
void UpdateHardwareInput () {
    rps.Update ();
    UpdateCDS ();
    UpdateBumps ();
}
// updates the hardware output values to the values that are stored in the vehicle object
void UpdateHardwareOutput () {
    UpdateMotors ();
    UpdateServos ();
}

// prints the motor percents corresponding to each motor
void PrintMotorPercents () {
    /*
     cout << "Motor 1: " << vehicle.wheels[0].activePercent << "%" << endl;
     cout << "Motor 2: " << vehicle.wheels[1].activePercent << "%" << endl;
     cout << "Motor 3: " << vehicle.wheels[2].activePercent << "%" << endl;
     */
}

// gets a random primary color
int getRandomColor () {
    int randomColorAmount = 6; // 3
    
    int randomNum = rand () % randomColorAmount;
    int color = BLACK;
    
    switch (randomNum) {
        case 0:
            color = RED;
            break;
        case 1:
            color = GREEN;
            break;
        case 2:
            color = BLUE;
            break;
        case 3:
            color = MAGENTA;
            break;
        case 4:
            color = CYAN;
            break;
        case 5:
            color = YELLOW;
            break;
        default:
            break;
    }
    
    return color;
}
// checks if strobe mode is enabled; if it is, sets the font color to a random color
void SetRandomColorIfStrobeMode () {
    if (strobeMode) {
        int color = getRandomColor ();
        LCD.SetFontColor (color);
    }
}
