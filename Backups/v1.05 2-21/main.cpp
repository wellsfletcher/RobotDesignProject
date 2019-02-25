#define IS_SIMULATION 1 // also have to define this variable in Simulation.hpp and Classes.hpp
#if IS_SIMULATION // if this is the simulation, then this chunck of code is used. Otherwise, this code is ignored.
/*
 
 
 
 SIMULATION only stuff
 
 
 
 */






// #include "GFWLCD.hpp"
// #include "GFWUtility.hpp" // now lives in "Class.hpp"
#include "GFWFunctions.hpp"
#include "GFWMotor.hpp" // #include "Simulation.hpp" // includes Classes.hpp // includes GFWLCD.hpp // #include "GFWUtility.hpp"
#include "GFWIO.hpp"
#include "GFWServo.hpp"
// #include "Classes.hpp"

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



// #define ZOOM_SCALE 3 // now lives in GFWLCD.hpp
void Init ();
void mainLoop ();
void DrawEverything ();
bool leftKeyInp;
bool upKeyInp;
bool rightKeyInp;
bool downKeyInp;

#define CALLBACK_DELAY .0001 // in seconds
#define UPDATE_INTERVAL 0.005
// GLint TIMER_DELAY = CALLBACK_DELAY * 1000; // in milleseconds
GLint TIMER_DELAY = 5; // in milleseconds

void Reshape(int w, int h) {
    glViewport (0, 0, w, h);                    // update the viewport
    glMatrixMode(GL_PROJECTION);                // update projection
    glLoadIdentity();
    gluOrtho2D(0.0, 320, 0.0, 240);             // map unit square to viewport
    glMatrixMode(GL_MODELVIEW);
    // glutPostRedisplay ();                       // request redisplay
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
        // cout << "Touch Y: " << LCD.touchY;
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
    glutCreateWindow("Game");                  // create the window
    
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
 
// #include <FEHLCD.h> // this statement lives in Classes.hpp
// #include <FEHUtility.h> // this statement lives in Classes.hpp
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include "Simulation.hpp"


// priority function declarations
void Init ();
void mainLoop ();
 
// code control variables
float timeSinceLastCodeCall; // amount of time alotted since the mainLoop code was last called
#define TIMER_DELAY 0.00 // delay before the mainLoop is called; 0 means that the mainloop code is called as soon as it is able to
#define UPDATE_INTERVAL 0.5

// Don't add anything to this main method. If you do, it will mess up the simulation code's behavior.
// Only add stuff to the initialization method (Init) and the main looping method (mainLoop).
// It shouldn't be necessary to add to the main method itself.
int main (void) {
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
void DrawEdge (Edge edge);
void DrawCircle (Circle circle);
void UpdateMotors ();
void UpdateBumps ();
void UpdateHardwareInput ();
void UpdateHardwareOutput ();
void PrintMotorPercents ();

// bump identifiers
/*
 #define F1 0
 #define B0 1
 #define B1 2
 #define D0 3
 #define D1 4
 #define F0 5
 */
#define F1 2
#define B0 3
#define B1 4
#define D0 5
#define D1 0
#define F0 1

/************************ HARDWARE VARIABLES ************************/

FEHMotor motor0 (FEHMotor::Motor0, 9.0); // the left motor
FEHMotor motor1 (FEHMotor::Motor1, 9.0); // the front motor
FEHMotor motor2 (FEHMotor::Motor2, 9.0); // the right motor

DigitalInputPin bump0 (FEHIO::P1_0);
DigitalInputPin bump1 (FEHIO::P1_1);
DigitalInputPin bump2 (FEHIO::P1_2);
DigitalInputPin bump3 (FEHIO::P1_3);
DigitalInputPin bump4 (FEHIO::P1_4);
DigitalInputPin bump5 (FEHIO::P1_5);

FEHServo servo0 (FEHServo::Servo0);
AnalogInputPin cds0 (FEHIO::P0_0);


/************************************************************************ CLASS DECLERATIONS ************************************************************************/

class Course {
public:
    Course (Vector2 globalPos, float pixelScale) { // 116–117 px  ~  1 ft
        PIXEL_SCALE = pixelScale;
        INCHES_TO_PIXELS = 9.72222222222 * PIXEL_SCALE;
        pos = globalPos;
        bounds = Box (Vector2 (0, 0), 350, 700);
        // WIDTH = 350;
        // HEIGHT = 700;
        bumb = Box (Vector2 (1, -340), 232, 11);
        discoBall = Circle (Vector2(155.5, -161), 25);
        obstacleBox = Box (Vector2 (174, -351), 59, 39); // 175
        
        DDRPad = Box (Vector2 (214, -569), 97, 48);
        DDRButtons = Box (Vector2 (214, -636), 97, 48);
        rightRamp = Box (Vector2 (232, -258), 116, 240);
        
        foosball = Box (Vector2 (155, -2), 166, 39);
        leverBank = Edge (Vector2 (1, -124), Vector2 (123, -2), 0);
        tokenSlot = Box (Vector2 (116, -351), 59, 39);
        tokenBowl = Circle (Vector2(146, -403.5), 22);
        
        leftRamp = Box (Vector2 (1, -351), 115, 87);
        finalBank = Edge (Vector2 (1, -617), Vector2 (83, -699), 0);
        
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
    // int WIDTH;
    // int HEIGHT;
    
    // draws the course
    void DrawCourse () {
        LCD.SetFontColor (GRAY);
        DrawCourseObject (bounds);
        
        LCD.SetFontColor (LIGHTGRAY);
        DrawCourseObject (bumb);
        DrawCourseObject (discoBall);
        DrawCourseObject (obstacleBox);
        
        LCD.SetFontColor (LIGHTGRAY);
        // LCD.SetFontColor (RED);
        DrawCourseObject (DDRPad);
        
        // LCD.SetFontColor (ORANGE);
        DrawCourseObject (DDRButtons);
        
        // LCD.SetFontColor (YELLOW);
        DrawCourseObject (rightRamp);
        
        // LCD.SetFontColor (LIMEGREEN);
        DrawCourseObject (foosball);
        
        // LCD.SetFontColor (BLUE);
        DrawCourseObject (leverBank);
        
        // LCD.SetFontColor (INDIGO);
        DrawCourseObject (tokenSlot);
        DrawCourseObject (tokenBowl);
        
        // LCD.SetFontColor (VIOLET);
        DrawCourseObject (leftRamp);
        
        // LCD.SetFontColor (PINK);
        DrawCourseObject (finalBank);
    }
    
    // draws a box object within the course
    void DrawCourseObject (Box obj) {
        // convert the box to global coordinates
        Box globalObj = Box ( ToGlobalPosition (obj.points[0]), obj.width * PIXEL_SCALE, obj.height * PIXEL_SCALE);
        // draw the global box through normal means
        DrawBox (globalObj);
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
        // LCD.SetFontColor (BLACK);
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
        
        // LCD.SetFontColor (BLACK);
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
            
            float scalar = 5.0;
            
            if (!vehicle.bumps [k].Value ()) {
                scalar *= 3.0;
            }
            
            Vector2 scaledDir = Vector2 (vehicle.bumps [k].dir.x * scalar, vehicle.bumps [k].dir.y * scalar);
            
            if (k == 5) { // the last one
                LCD.SetFontColor (BLUE);
            }
            
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
            
            PlotVector (global, scaledDir);
        }
    }
    // draws real vehicle vectors (wheel direction, vehicle movement direction)
    void DrawVehicleVectors (Vehicle vehicle) {
        float scaler = 50.0;
        // draw vectors for each wheel
        for (int k = 0; k < vehicle.wheelsLength; k++) {
            Vector2 globalWheelPos = Vector2 (vehicle.wheels [k].pos.x + vehicle.pos.x, vehicle.wheels [k].pos.y + vehicle.pos.y);
            // Vector2 scaledDir = Vector2 (vehicle.wheels [k].dir.x * scaler, vehicle.wheels [k].dir.y * scaler);
            Vector2 scaledDir = Vector2 (vehicle.wheels [k].lastForceApplied.x * scaler, vehicle.wheels [k].lastForceApplied.y * scaler);
            PlotVector (globalWheelPos, scaledDir);
        }
        // draw the vehicle vector direction
        // Vector2 scaledDir = Vector2 (vehicle.dir.x * scaler, vehicle.dir.y * scaler);
        Vector2 scaledDir = Vector2 (vehicle.lastForceApplied.x * scaler, vehicle.lastForceApplied.y * scaler);
        PlotVector (vehicle.pos, scaledDir);
    }
    
    // conpoints a local position relative to the course to a global position
    Vector2 ToGlobalPosition (Vector2 local) {
        Vector2 global = Vector2 (local.x * PIXEL_SCALE + pos.x, local.y * PIXEL_SCALE + pos.y);
        return global;
    }
    // conpoints a local position relative to the course to a global position
    Vector2 ToRealGlobalPosition (Vector2 local) {
        Vector2 global = Vector2 (local.x * INCHES_TO_PIXELS + pos.x, local.y * INCHES_TO_PIXELS + pos.y);
        return global;
    }
private:
};


class Navigator {
public:
    Navigator (Vehicle *vehicle) {
        veh = vehicle;
        timeSinceAction = TimeNow ();
        state = 0;
        actionToPerform = 0;
        
        timeWhenRotationStarted = 0;
        timeRequiredForRotation = 0;
        rotationDidNotJustStart = false;
    }
    Navigator () {
        
    }
    Vehicle *veh;
    int state;
    int actionToPerform;
    float timeSinceAction;
    
    float timeWhenRotationStarted;
    float timeRequiredForRotation;
    bool rotationDidNotJustStart;
    
    // turn a ceratin amount of degrees; must be called repeadetly from a loop; returns true once the turn is complete
    bool TurnDegrees (float degrees, float motorPercent) {
        float ANG_VELOCITY_AT_50_POWER = 30; // in degrees per second
        if (rotationDidNotJustStart == false) {
            veh->Turn (motorPercent);
            float angVel = ANG_VELOCITY_AT_50_POWER * 2.0 * motorPercent / 100.0;
            // timeRequiredForRotation = abs (degrees / angVel); // t = Ø / w; may remove abs in the future
            timeRequiredForRotation = degrees / angVel;
            timeWhenRotationStarted = TimeNow ();
            rotationDidNotJustStart = true;
            // cout << "delta t = " << timeRequiredForRotation << endl;
        }
        if (TimeNow () - timeWhenRotationStarted > timeRequiredForRotation) {
            // cout << "elapsed t = " << TimeNow () - timeWhenRotationStarted << endl;
            rotationDidNotJustStart = false;
            veh->Turn (-motorPercent); // "remove" the turn from the motors
            return true;
        } else {
            return false;
        }
    }
    
    /************************ TASK FUNCTIONS ************************/
    
    void StopVehicle (float stopTime) {
        veh->Stop ();
        if (TimeNow() - timeSinceAction > stopTime) {
            state++;
            timeSinceAction = TimeNow ();
        }
    }
    
    void TurnCW (float degrees, float power) {
        if (TurnDegrees (-degrees, -power)) {
            state++;
            timeSinceAction = TimeNow ();
        }
    }
    
    void TurnCCW (float degrees, float power) {
        if (TurnDegrees (degrees, power)) {
            state++;
            timeSinceAction = TimeNow ();
        }
    }
    
    void TurnUntilBumpCW (int bumpID, float power, float maxDegrees) {
        // veh->Turn (-power);
        TestEndCondition (TurnDegrees (-maxDegrees, -power) || !veh->bumps [bumpID].Value());
    }
    
    void TurnUntilBumpCCW (int bumpID, float power, float maxDegrees) {
        // veh->Turn (-power);
        TestEndCondition (TurnDegrees (maxDegrees, power) || !veh->bumps [bumpID].Value());
    }
    
    void MoveDuration (Vector2 direction, float power, float duration) {
        veh->Move (direction, power);
        if (TimeNow() - timeSinceAction > duration) {
            state++;
            timeSinceAction = TimeNow ();
        }
    }
    
    void MoveBump (Vector2 direction, int bumpID, float power) {
        veh->Move (direction, power / 2.0);
        if (!veh->bumps [bumpID].Value()) {
            state++;
            timeSinceAction = TimeNow ();
        }
    }
    
    void SetServoAngle (float degrees, float waitTime) {
        veh->servo.SetDegree (degrees);
        TestEndCondition (TimeNow() - timeSinceAction > waitTime);
    }
    
    void WaitForLight () {
        TestEndCondition (veh->cds.isLight());
    }
    
    void TestEndCondition (bool conditionIsTrue) {
        // if condition is true, then increment the current state (in order to move onto the next action) and update the timeSinceAction
        if (conditionIsTrue) {
            state++;
            timeSinceAction = TimeNow ();
        }
    }
    
    // task variable definitions
    static const int DO_NOTHING = 0;
    static const int STOP = 1;
    static const int WAIT_FOR_START_LIGHT = 100;
    
    static const int TURN_45_CW = 2;
    static const int MOVE_E_DURATION = 14;
    
    static const int MOVE_DE_DURATION = 3;
    static const int MOVE_DE_BUMP_D0 = 4;
    
    static const int MOVE_FA_DURATION = 5;
    static const int MOVE_FA_BUMP_F1 = 6;
    
    static const int TURN_CW_BUMP_F0 = 7;
    static const int GRIND_UP_RAMP = 8;
    
    static const int MOVE_E_BUMP_D1 = 9;
    static const int MOVE_AB_DURATION = 10;
    static const int MOVE_C_BUMP_D0 = 11;
    static const int TURN_CW_BUMP_B1 = 12;
    static const int MOVE_F_DURATION = 13;
    
    static const int LOWER_SERVO = 50;
    
    static const int END = 999;
    
    // performs the series of actions required for performance test one; must be repeatedly called from a loop; is draft 2
    void PerformanceTestOneD2 () {
        UpdateHardwareInput ();
        
        int sequence [] = {WAIT_FOR_START_LIGHT, TURN_45_CW, MOVE_E_DURATION, MOVE_FA_DURATION, MOVE_FA_BUMP_F1, TURN_CW_BUMP_F0, STOP, GRIND_UP_RAMP, MOVE_E_BUMP_D1, MOVE_AB_DURATION, MOVE_C_BUMP_D0, TURN_CW_BUMP_B1, STOP, MOVE_F_DURATION, STOP, LOWER_SERVO, STOP, END}; // currently, the first and last items should always be DO_NOTHING and END
        actionToPerform = sequence [state];
        
        float timeScale = 0.2; // for testing purposes
        
        float power = 50; // note: power was at -35 (or maybe -25) for that speed video I recorded
        float stopTime = 1.0;
        Vector2 direction;
        
        float duration5 = 1.0 * timeScale; // short duration at the beginning spent moving forward
        float duration0 = 3.0 * timeScale; // time spent initially going right
        float duration1 = 5.0 * timeScale; // time going at max power up the ramp
        float duration2 = 0.5 * timeScale; // short duration after colliding with the wall
        float duration3 = 0.5 * timeScale; // short duration spent backing up from lever
        float duration4 = 2.5 * timeScale; // duration spent waiting for the servo to lower

        
        switch (actionToPerform) {
            case DO_NOTHING: {
                timeSinceAction = TimeNow ();
            } break;
                
            case WAIT_FOR_START_LIGHT: {
                WaitForLight ();
                veh->SetPosition (  Vector2 (9,-63)  );
            } break;
                
            case STOP: { // turn 45 degrees CW
                StopVehicle (stopTime);
            } break;
               
            case MOVE_E_DURATION: {
                MoveDuration (Vector.E, power * 0.75, duration5);
            } break;
                
            case TURN_45_CW: {
                TurnCW (45, power);
            } break;
                
            case MOVE_DE_DURATION: { // move in DE direction (rightwards)
                MoveDuration (Vector.DE, power, duration0);
            } break;
                
            case MOVE_DE_BUMP_D0: { // move in DE direction (rightwards) but slower, and stop once the bump switch is pressed
                MoveBump (Vector.DE, F0, power / 4.0);
            } break;
                
            case MOVE_FA_DURATION: { // move in DE direction (rightwards)
                MoveDuration (Vector.FA, power, duration0);
            } break;
                
            case MOVE_FA_BUMP_F1: { // move in DE direction (rightwards) but slower, and stop once the bump switch is pressed
                MoveBump (Vector.FA, F1, power / 4.0);
            } break;
                
            case TURN_CW_BUMP_F0: { // note that the vehicle does not necessarily stop
                float degreeTurnLimit = 90;
                TurnUntilBumpCW (F0, power, degreeTurnLimit);
                veh->SetPosition (  Vector2 (32, -60)  );
            } break;
                
            // go up the ramp
            case GRIND_UP_RAMP: {
                MoveDuration (Vector.E, power * 1.7, duration1);
                // may also want to implement a thing here to make the vehicle stay flush against the wall
            } break;
                
            case MOVE_E_BUMP_D1: { // move in DE direction (rightwards) but slower, and stop once the bump switch is pressed
                MoveBump (Vector.E, D1, power / 2.0);
                // may also want to implement a thing here to make the vehicle stay flush against the wall
            } break;
        
            // move back from the wall
            case MOVE_AB_DURATION: { // move back a little
                MoveDuration (Vector.AB, power / 4.0, duration2);
                veh->SetPosition (  Vector2 (32, -8)  );
            } break;
        
            case MOVE_C_BUMP_D0: { // C0 == D0
                MoveBump (Vector.C, D0, power / 1.5);
            } break;
        
            // try to get flush with the lever
            case TURN_CW_BUMP_B1: { // getting flush with the lever; note that the vehicle does not necessarily stop
                float degreeTurnLimit = 45;
                TurnUntilBumpCW (B1, power, degreeTurnLimit); // C1 = B1
                veh->SetPosition (  Vector2 (12, -12)  );
            } break;
        
            case MOVE_F_DURATION: { // short duration spent backing up from lever
                MoveDuration (Vector.F, power / 4.0, duration3);
            } break;
        
            // lower the servo
            case LOWER_SERVO: {
                SetServoAngle (0, duration4);
            } break;
        
            case END: {
                actionToPerform = 0;
            } break;
                
            default: {
                // ...
            } break;
        }
        UpdateHardwareOutput ();
    }
    // performs the series of actions required for performance test one; must be repeatedly called from a loop; is draft 1
    void PerformanceTestOneD1 () {
        UpdateHardwareInput ();
        
        float timeScale = 0.2; // for testing purposes
        
        float power = 50; // note: power was at -35 (or maybe -25) for that speed video I recorded
        float stopTime = 1.0;
        Vector2 direction;
        
        float duration0 = 3.0 * timeScale;
        float duration1 = 10.0 * timeScale;
        float duration2 = 10.0 * timeScale;
        
        switch (state) {
            case 0: {
                timeSinceAction = TimeNow ();
            } break;
            case 1: { // turn 45 degrees CW
                // cout << Vector.D.getAngle() << endl;
                if (TurnDegrees (-45, -power)) {
                    state++;
                    timeSinceAction = TimeNow ();
                }
            } break;
            case 2: { // stop the vehicle
                veh->Stop ();
                if (TimeNow() - timeSinceAction > stopTime) {
                    state++;
                    timeSinceAction = TimeNow ();
                }
            } break;
            case 3: { // move in DE direction (rightwards)
                veh->Move (Vector.DE, power);
                if (TimeNow() - timeSinceAction > duration0) {
                    state++;
                    timeSinceAction = TimeNow ();
                }
            } break;
            case 4: { // move in DE direction (rightwards) but slower, and stop once the bump switch is pressed
                veh->Move (Vector.DE, power / 2.0);
                if (!veh->bumps [D1].Value()) {
                    state++;
                    timeSinceAction = TimeNow ();
                }
            } break;
                /*
            case 5: {
                veh->Turn (power / 2.0);
                if (!veh->bumps [D0].Value()) {
                    state++;
                    timeSinceAction = TimeNow ();
                }
            } break;
            case 6: {
                veh->Stop ();
                if (TimeNow() - timeSinceAction > stopTime) {
                    state++;
                    timeSinceAction = TimeNow ();
                }
            } break;
            case 7: {
                veh->Move (Vector.F, power / 2.0);
                if (veh->bumps [69].Value()) {
                    state++;
                    timeSinceAction = TimeNow ();
                }
            } break;
                 */
            case 5: {
                veh->Stop ();
                if (TimeNow() - timeSinceAction > stopTime) {
                    state = 0;
                    timeSinceAction = TimeNow ();
                }
            } break;
            default: {
                // ...
            } break;
        }
        UpdateHardwareOutput ();
    }
    // performs vehicle test procedure in which the vehicle drives forward; must be repeatedly called from a loop
    void ForwardTest () {
        UpdateHardwareInput ();
        
        float power = -80; // note: power was at -35 (or maybe -25) for that speed video I recorded
        float timeBetween = 10.0;
        Vector2 direction;
        
        switch (state) {
            case 0:
                timeSinceAction = TimeNow ();
                break;
            case 1:
                veh->Move (Vector.right, power);
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 2;
                    timeSinceAction = TimeNow ();
                }
                break;
            case 2:
                veh->Stop ();
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 0;
                    timeSinceAction = TimeNow ();
                }
                break;
            default:
                break;
        }
        UpdateHardwareOutput ();
    }
    // performs vehicle test procedure in which the drives in a square and then a diagonal; must be repeatedly called from a loop
    void SquareAndDiagonalTest () {
        UpdateHardwareInput ();
        
        float power = 25; // note: power was at -35 (or maybe -25) for that speed video I recorded
        float timeBetween = 0.5;
        Vector2 direction;
        
        switch (state) {
            case 0:
                timeSinceAction = TimeNow ();
                break;
            case 1:
                veh->Move (Vector.right, power);
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 2;
                    timeSinceAction = TimeNow ();
                }
                break;
            case 2:
                veh->Stop ();
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 3;
                    timeSinceAction = TimeNow ();
                }
                break;
            case 3:
                veh->Move (Vector.up, power);
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 4;
                    timeSinceAction = TimeNow ();
                }
                break;
            case 4:
                veh->Stop ();
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 5;
                    timeSinceAction = TimeNow ();
                }
                break;
            case 5:
                veh->Move (Vector.left, power);
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 6;
                    timeSinceAction = TimeNow ();
                }
                break;
            case 6:
                veh->Stop ();
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 7;
                    timeSinceAction = TimeNow ();
                }
                break;
            case 7:
                veh->Move (Vector.down, power);
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 8;
                    timeSinceAction = TimeNow ();
                }
                break;
            case 8:
                veh->Stop ();
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 9;
                    timeSinceAction = TimeNow ();
                }
                break;
            case 9:
                direction = Vector2 (-1, -1);
                direction = direction.getUnitVector ();
                veh->Move (direction, power);
                if (TimeNow() - timeSinceAction > timeBetween * 1.05) {
                    state = 10;
                    timeSinceAction = TimeNow ();
                }
                break;
            case 10:
                veh->Stop ();
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 11;
                    timeSinceAction = TimeNow ();
                }
                break;
            case 11:
                direction = Vector2 (1, 1);
                direction = direction.getUnitVector ();
                veh->Move (direction, power);
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 12;
                    timeSinceAction = TimeNow ();
                }
                break;
            case 12:
                veh->Stop ();
                if (TimeNow() - timeSinceAction > timeBetween) {
                    state = 0;
                    timeSinceAction = TimeNow ();
                }
                break;
            default:
                break;
        }
        UpdateHardwareOutput ();
    }
    // resets the navigator object (resets state variables and that sort of thing)
    void Reset () {
        state = 0;
        timeSinceAction = TimeNow ();
    }
    // starts the navigation procedure; does this by setting state = 1
    void Start () {
        state = 1;
    }
    Vectors Vector;
private:
};


/************************************************ MISCELLANEOUS CLASSES ************************************************/
/************************************************************************ FUNCTION DECLERATIONS ************************************************************************/
/************************************************************************ GLOBAL VARIABLE DECLERATIONS ************************************************************************/


/************************ SETTINGS VARIABLES ************************/

// ...


/************************ MISCELLANEOUS VARIABLES ************************/

Vector2 touch = Vector2 ();
bool touchState = false;
float timeSinceLastFrameUpdate;

Course course;
Vehicle vehicle;
Simulation simulation;
Navigator navigator;

Vectors Vector;


/************************ MISCELLANEOUS VARIABLES ************************/

// define bump identifiers


/************************ GUI VARIABLES ************************/

// sample variables for button declaration
#define BUTT_WIDTH 100
#define BUTT_HEIGHT 30
Button butt;
Button cancelButt;
Button actionButt0;
Button actionButt1;


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
    // array global variable initializations
    const double sqrt3 = pow (3.0, 0.5);
    
    // sample button declaration
    char charPtr [100];
    strcpy (charPtr, "START");
    butt = Button (  Box ( Vector2 (30, -60), BUTT_WIDTH, BUTT_HEIGHT ), charPtr, 0  );
    
    strcpy (charPtr, "RESET");
    cancelButt = Button (  Box ( Vector2 (30, -105), BUTT_WIDTH, BUTT_HEIGHT ), charPtr, 0  );
    
    strcpy (charPtr, "ACT");
    actionButt0 = Button (  Box ( Vector2 (30, -150), 45, BUTT_HEIGHT ), charPtr, 0  );
    actionButt1 = Button (  Box ( Vector2 (85, -150), 45, BUTT_HEIGHT ), charPtr, 0  );
    
    // initialize the course object
    course = Course (Vector2 (150, -32), .25);
    
    int reverse = -1;
    int wheelsLength = 3;
    int bumpsLength = 6;
    float radius = 1.2;
    float depth = 1.6;
    float hexSide = 4.25;
    float lowerHexRadius = (2.0*sqrt3 + 1.5*sqrt3) / 2.0; // (2*3^.5 + 1.5*3^.5) / 2.0 = 6.06217782649 / 2.0 ~= 3
    Vector2 lowerDisp = Vector2 (0, 1.0); // displacement between the lower chassis and the upper chassis
    Wheel leftWheel = Wheel ( Vector2(lowerDisp.x - lowerHexRadius, lowerDisp.y - lowerHexRadius), Vector2(1 * reverse,-sqrt3 * reverse), radius, depth );
    Wheel frontWheel = Wheel ( Vector2(lowerDisp.x, lowerDisp.y + lowerHexRadius), Vector2(-1 * reverse, 0), radius, depth );
    Wheel rightWheel = Wheel ( Vector2(lowerDisp.x + lowerHexRadius, lowerDisp.y - lowerHexRadius), Vector2(1 * reverse,sqrt3 * reverse), radius, depth );
    
    Wheel wheels[] = {   leftWheel, frontWheel, rightWheel  }; // 2.75 from number friendly center
    
    BumpSwitch bump0 = BumpSwitch (Vector2 (-hexSide/2.0, -hexSide), Vector2 (0, -1.0));
    BumpSwitch bump1 = BumpSwitch (Vector2 (-hexSide, 0), Vector2 (-sqrt3, 1));
    BumpSwitch bump2 = BumpSwitch (Vector2 (-hexSide/2.0, hexSide), Vector2 (-sqrt3, 1));
    BumpSwitch bump3 = BumpSwitch (Vector2 (hexSide/2.0, hexSide), Vector2 (sqrt3, 1));
    BumpSwitch bump4 = BumpSwitch (Vector2 (hexSide, 0), Vector2 (sqrt3, 1));
    BumpSwitch bump5 = BumpSwitch (Vector2 (hexSide/2.0, -hexSide), Vector2 (0, -1.0));
    
    BumpSwitch bumps[] = {   bump0, bump1, bump2, bump3, bump4, bump5   };
    
    float servoStartDegrees = 0;
    int servoMin = 500;
    int servoMax = 2500;
    servo0.SetMin (servoMin);
    servo0.SetMax (servoMax);
    Servo servo = Servo (Vector2 (0, 0.5), Vector2 (0, 1.0), servoStartDegrees);
    
    CDS cds = CDS (Vector2 (0, 1.0), Vector2 (0, 1.0));
    
    Vector2 centerOfMass = Vector2 (0,0);
    Vector2 startPosition = Vector2 (9,-63);
    Vector2 startDirection = Vector2 (0,1);
    Vector2 chPoints[6] = {Vector2 (-hexSide/2.0, hexSide),  Vector2 (hexSide/2.0, hexSide),  Vector2 (hexSide, 0),  Vector2 (hexSide/2.0, -hexSide),  Vector2 (-hexSide/2.0, -hexSide),  Vector2 (-hexSide, 0)};
    Polygon chassis = Polygon (chPoints, 6);
    vehicle = Vehicle (startPosition, startDirection, centerOfMass, chassis, servo, cds, wheels, wheelsLength, bumps, bumpsLength, lowerHexRadius);
    // vehicle.SetRotation (Vector2 (-1, -1)); // since the direction of the wheel are reversed, it means that — in effect – the front and back sides are flipped; now the side with no wheels is the front, which is good because that's the side we wanted to change to the front any way
    vehicle.AddRotation (255); // 180-45+120
    
    if (IS_SIMULATION) {
        simulatedVehicle = Vehicle (vehicle); // create a vehicle object that shows where the vehicle would by according to simulated values; this should exist in the Xcode simulation
        simulation = Simulation (&simulatedVehicle);
    }
    navigator = Navigator (&vehicle);
    // cout << "before: " << navigator.Vector.D.x << ", " << navigator.Vector.D.y << endl;
    navigator.Vector.AlignVehicleVectors (75); // 60-120                   // ******* important this might need to be 60 instead of 75 *******
    Vector.AlignVehicleVectors (75);
    // cout << "after: " << navigator.Vector.D.x << ", " << navigator.Vector.D.y << endl;
    
    timeSinceLastFrameUpdate = TimeNow (); // initialize the time since the last screen/frame update
}


// draws everything; all drawing functions should go here
// It's important to put all LCD functions here. It's critical for the simulation to work properly and
// is good coding practice for graphical computing, especially when you have limited computing capactity
// like you do on the Proteus.
void DrawEverything () {
    LCD.Clear (BACKGROUNDCOLOR); // clear the screen (for animation purposes)
    
    // draw the course
    course.DrawCourse ();
    
    if (IS_SIMULATION) {
        LCD.SetFontColor (BLACK);
        course.DrawRealVehicle (simulatedVehicle);
        LCD.SetFontColor (RED);
        course.DrawVehicleVectors (simulatedVehicle);
    }
    
    LCD.SetFontColor (TURQUOISE);
    course.DrawRealVehicle (vehicle);
    
    // sample button drawing function implementation
    butt.DrawButton();
    cancelButt.DrawButton();
    actionButt0.DrawButton();
    actionButt1.DrawButton();
}


// do all of the stuff for the main game loop that gets repeated
void mainLoop () {
    if (IS_SIMULATION) {
        // simulatedVehicle.Update ();
        simulation.Update ();
    }
    touchState = LCD.Touch (&touch.x, &touch.y); // get the user's touch input coordinates, and store whether or not the user is touching the screen in the variable touchState
    touch = Vector2 (touch.x, -touch.y); // standardize the touch coordinates so they're effectively in the fourth quadrant
    
    // perform all game actions (i.e. check for inputs, calculate physics, etc.) except for updating the screen
    
    if (actionButt0.WasPressed (touch, touchState)) { // checks if the button was pressed and updates the state of the button
        vehicle.servo.SetDegree (0);
    }
    if (actionButt1.WasPressed (touch, touchState)) { // checks if the button was pressed and updates the state of the button
        vehicle.servo.SetDegree (180);
    }
    
    if (cancelButt.WasPressed (touch, touchState)) { // checks if the button was pressed and updates the state of the button
        navigator.Reset ();
    }
    if (butt.IsBeingPressed (touch)) { // checks if the button was pressed and updates the state of the button
        // vehicle.AddRotation (5);
        navigator.Start ();
    }
    
    navigator.PerformanceTestOneD2 ();
    
    
    // update the screen only after a fixed time interval has passed
    if (TimeNow () - timeSinceLastFrameUpdate > UPDATE_INTERVAL) { // I've heard TimeNow is bad, so the game may break after a certain amount of time has passed
        timeSinceLastFrameUpdate = TimeNow ();
        
        // basically just calls the DrawEverything function, which draws everything and updates the screen
        glutPostRedisplay (); // SIMULATION code chunk
        // DrawEverything (); // NON-simulation code chunk
    }
    
}


// draws a box with the given box object
void DrawBox (Box box) {
    LCD.FillRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y), box.width, box.height);
}

// draws given edge
void DrawEdge (Edge edge) {
    LCD.DrawLine (  (int)(edge.points[0].x), -(int)(edge.points[0].y), (int)(edge.points[1].x), -(int)(edge.points[1].y)  );
}

// draws given cricle
void DrawCircle (Circle circle) {
    LCD.FillCircle (  (int)(circle.points[0].x), -(int)(circle.points[0].y), circle.radius  );
}

// draws given cricle
void DrawPolygon (Polygon poly) {
    for (int k = 0; k < poly.length-1; k++) {
        LCD.DrawLine (  (int)(poly.points[k].x), -(int)(poly.points[k].y), (int)(poly.points[k+1].x), -(int)(poly.points[k+1].y)  );
    }
    LCD.DrawLine (  (int)(poly.points[poly.length].x), -(int)(poly.points[poly.length].y), (int)(poly.points[0].x), -(int)(poly.points[0].y)  );
}


void UpdateCDS () {
    vehicle.cds.value = cds0.Value ();
}
void UpdateServos () {
    servo0.SetDegree (vehicle.servo.degrees);
}
void UpdateMotors () {
    motor0.SetPercent (vehicle.wheels[0].activePercent);
    motor1.SetPercent (vehicle.wheels[1].activePercent);
    motor2.SetPercent (vehicle.wheels[2].activePercent);
}
void UpdateBumps () {
    vehicle.bumps [0].value = bump0.Value ();
    vehicle.bumps [1].value = bump1.Value ();
    vehicle.bumps [2].value = bump2.Value ();
    vehicle.bumps [3].value = bump3.Value ();
    vehicle.bumps [4].value = bump4.Value ();
    vehicle.bumps [5].value = bump5.Value ();
}
void UpdateHardwareInput () {
    UpdateCDS ();
    UpdateBumps ();
}
void UpdateHardwareOutput () {
    UpdateMotors ();
    UpdateServos ();
}
void PrintMotorPercents () {
    /*
    cout << "Motor 1: " << vehicle.wheels[0].activePercent << "%" << endl;
    cout << "Motor 2: " << vehicle.wheels[1].activePercent << "%" << endl;
    cout << "Motor 3: " << vehicle.wheels[2].activePercent << "%" << endl;
    */
}