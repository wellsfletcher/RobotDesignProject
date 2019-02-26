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
// #include <vector> // if this doesn't work, I'm just gonna make a giant ass array


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
        bounds = Box (Vector2 (0, 0), 350, 700);

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

    /************************ these functions (Draw Course Object) draw shapes where their coordinates are in PIXELS relative to the top left corner of the course rendering. They are also scaled by the PIXEL_SCALE constant. ************************/
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
            float scalar = 5.0;
            if (!vehicle.bumps [k].Value ()) {
                scalar *= 3.0;
            }

            Vector2 scaledDir = Vector2 (vehicle.bumps [k].dir.x * scalar, vehicle.bumps [k].dir.y * scalar);

            // make the last bump switch blue in color
            if (k == 5) { // the last one
                LCD.SetFontColor (BLUE);
            }

            // draw bump switch labels
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

            // draw bump switch direction indicators
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




/************************************************************************ NAVIGATION CLASS DECLERATIONS ************************************************************************/


/************************ MISC DECLARATIONS ************************/

class Navigator;


/************************ STATE DECLARATIONS ************************/

// this is the base class for the states used in the state machine object
class State {
public:
    State () {
        
    }
    
    // perform the state's actions
    virtual const bool Go () {
        // cout << "Ruh roh" << endl;
        return false;
    }
    // int testValue;
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
    virtual const bool Go () {
        return (nav->*functionPtr) (); // calls the function in which functionPtr is pointing to
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
    virtual const bool Go () {
        return (nav->*functionPtr) (v0, f0, f1, i0, i1); // calls the function in which functionPtr is pointing to
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
    virtual const bool Go () {
        return (nav->*functionPtr) (v0, f0, f1, i0); // calls the function in which functionPtr is pointing to
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
    virtual const bool Go () {
        return (nav->*functionPtr) (v0, f0, f1); // calls the function in which functionPtr is pointing to
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
    virtual const bool Go () {
        return (nav->*functionPtr) (v0, f0, i0); // calls the function in which functionPtr is pointing to
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
    virtual const bool Go () {
        return (nav->*functionPtr) (f0, f1, i0); // calls the function in which functionPtr is pointing to
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
    virtual const bool Go () {
        return (nav->*functionPtr) (f0, f1); // calls the function in which functionPtr is pointing to
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
    virtual const bool Go () {
        return (nav->*functionPtr) (f0); // calls the function in which functionPtr is pointing to
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
        timeWhenStateChanged = TimeNow ();
    }
    // vector<State*> states; // state data vector
    State *states [64];
    int length; // the length of the state data vector
    int currentState;
    float timeWhenStateChanged;
    
    // adds an state/event/task
    void Add (State *newState) { // takes an State struct as an argument
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
    
    void Update () {
        // cout << "Go! " << currentState << endl;
        // call event data function until it returns true (the end condition is met)
        bool stateFinished = states [currentState]->Go ();
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

class Navigator {
public:
    Navigator (Vehicle *vehicle) {
        veh = vehicle;
        SM = StateMachine ();

        timeWhenRotationStarted = 0;
        timeRequiredForRotation = 0;
        rotationDidNotJustStart = false;

        leftTurnLimitReached = false;
        rightTurnLimitReached = false;
    }
    Navigator () {

    }
    Vehicle *veh;
    StateMachine SM;

    float timeWhenRotationStarted;
    float timeRequiredForRotation;
    bool rotationDidNotJustStart;

    // turn a ceratin amount of degrees; must be called repeadetly from a loop; returns true once the turn is complete
    bool TurnDegrees (float degrees, float motorPercent) {
        float ANG_VELOCITY_AT_50_POWER = 120; // in degrees per second // 150
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

    /*
     * Stops the vehicle by setting the percent power of all of the wheels to zero. Waits @stopTime in seconds.
     * @return true when the action is complete, false otherwise.
     */
    bool StopVehicle (float stopTime) {
        veh->Stop ();
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
        PrintMotorPercents ();
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
        // veh->Turn (-power);
        return (TurnDegrees (maxDegrees, power) || !veh->bumps [bumpID].Value());
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
        // veh->Turn (-power);
        return (TurnDegrees (-maxDegrees, -power) || !veh->bumps [bumpID].Value());
    }

    /*
     * Moves the vehicle in the given @direction with the given @power until the given @duration in seconds has passed.
     * @return true when the action is complete, false otherwise.
     */
    bool MoveDuration (Vector2 direction, float power, float duration) {
        PrintMotorPercents ();
        veh->Move (direction, power);
        return (TimeNow() - SM.timeWhenStateChanged > duration);
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
     * Moves the vehicle in the given @direction with the given @power until the bump switch with index @bumpID is activated or until the @maxDuration is reached.
     * @return true when the action is complete, false otherwise.
     */
    bool MoveUntilBumpDuration (Vector2 direction, float power, float maxDuration, int bumpID) {
        veh->Move (direction, power);
        return !veh->bumps [bumpID].Value() || (TimeNow() - SM.timeWhenStateChanged > maxDuration);
    }
    

    /*
     * Set the servos angle to @degrees, and waits for @waitTime in seconds until performing the next action.
     * @return true when the action is complete, false otherwise.
     */
    bool SetServoAngle (float degrees, float waitTime) {
        veh->servo.SetDegree (degrees);
        return (TimeNow() - SM.timeWhenStateChanged > waitTime);
    }
    
    /*
     * Does DDR stuff.
     */
    bool DoDDR () {
        return (TimeNow() - SM.timeWhenStateChanged > 69.420);
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

    // update the navigator; must be called repeatedly for everything to work properly
    void Update () {
        UpdateHardwareInput ();
        // SM.Update ();
        PerformanceTestTwoD1 (); // the current robot control procedure
        UpdateHardwareOutput ();
    }
    
    
    // perform all of the navigation procedures associated with state machine test
    void PerformanceTestTwoD1 () {
        /*
         * This is set up so that all the information except the state data itself is stored in the SM statemachine.
         * This includes the currentState and time when the state was last changed.
         * This non-state data is transfered into a temporary state machine (TSM), in which all the actual state data is then manually added.
         * This temporary state machine is then updated (which calls the function associated with the current state).
         * The non-state data in the tempory state machine is then transfered back into SM, so any state transition information is noted.
         * This is all really inefficent, but it is the easiest way to solve a memory leak issue I was having.
         */
        
        StateMachine TSM = StateMachine (SM.timeWhenStateChanged, SM.currentState); // transfer non-state data into a temporary state machine
        
        
        /************************ initialize SAMPLE state objects ************************/
        
        // FYI the (& Navigator :: FunctionName) syntax is used to create a pointer to a function existing within a navigator object; the spaces are for style
        State do_nothing = State (); // this syntax is different than the other because the compiler would otherwise think this is a method declaration (other than that, the syntax is essentially equivalent)
        StateVoid wait_for_Light (this, & Navigator :: WaitForLight);
        
        StateVFF move_duration (this, & Navigator :: MoveDuration, Vector.EF, 35, 1.0); // direction, power, duration
        StateVFI move_bump (this, & Navigator :: MoveUntilBump, Vector.D, 35, D0); // direction, power, bumpID
        StateVFFI move_bump_duration (this, & Navigator :: MoveUntilBumpDuration, Vector.D, 35, 3.0, D1);  // direction, power, duration, bumpID
        
        StateFF turn_clockwise (this, & Navigator :: TurnCW, 45, 35); // degrees, power
        StateFFI turn_clockwise_bump (this, & Navigator :: TurnUntilBumpCW, 45, 35, D0); // maxDegrees, power, bumpID
        
        StateVFFII move_while_flush (this, & Navigator :: MoveWhileFlushDuration, Vector.E, 35, 4.20, F0, F1); // direction, power, duration, bump0, bump1
        StateFF set_servo_angle (this, & Navigator :: SetServoAngle, 110, 1.0); // degrees, waitTime
        
        StateF stop (this, & Navigator :: StopVehicle, 1.0); // stopTime
        
        
        /************************ initialize OTHER state objects ************************/
        
        StateF stop_at_the_end (this, & Navigator :: StopVehicle, 0.01); // stopTime
        StateVFF move_up_ramp_real_quick (this, & Navigator :: MoveDuration, Vector.E, 65, 1.0); // direction, power, duration
        StateFF initial_turn (this, & Navigator :: TurnCW, 135, 35); // degrees, power
        StateVoid do_DDR_stuff (this, & Navigator :: DoDDR);
        StateVFF move_duration_DE (this, & Navigator :: MoveDuration, Vector.DE, 35, 1.0); // direction, power, duration
        
        
        /************************ ADD REFERENCES of all of the state objects to the temporary state machine ************************/
        
        // note you have to start this new state machine using the GUI button (or at least that's how it's currently set up)
        TSM.Add (  & do_nothing   ); // the do nothing state should always be the first state (state [0])
        TSM.Add (  & wait_for_Light   );
        
        TSM.Add (  & initial_turn   );
        TSM.Add (  & stop   );
        TSM.Add (  & move_duration_DE   );
        TSM.Add (  & stop   );
        TSM.Add (  & turn_clockwise   );
        TSM.Add (  & stop   );
        TSM.Add (  & do_DDR_stuff   );
        
        TSM.Add (  & stop_at_the_end   ); // probably always a good idea to include a stop function at the end
        
        
        TSM.Update ();
        
        SM = StateMachine (TSM.timeWhenStateChanged, TSM.currentState); // transfer tempary non-state data into the real state machine
    }
    
    // perform all of the navigation procedures associated with state machine test
    void StateMachineTest () {
        /* 
         * This is set up so that all the information except the state data itself is stored in the SM statemachine.
         * This includes the currentState and time when the state was last changed.
         * This non-state data is transfered into a temporary state machine (TSM), in which all the actual state data is then manually added.
         * This temporary state machine is then updated (which calls the function associated with the current state).
         * The non-state data in the tempory state machine is then transfered back into SM, so any state transition information is noted.
         * This is all really inefficent, but it is the easiest way to solve a memory leak issue I was having.
         */

        StateMachine TSM = StateMachine (SM.timeWhenStateChanged, SM.currentState); // transfer non-state data into a temporary state machine
        
        
        /************************ initialize SAMPLE state objects ************************/
        
        // FYI the (& Navigator :: FunctionName) syntax is used to create a pointer to a function existing within a navigator object; the spaces are for style
        State do_nothing = State (); // this syntax is different than the other because the compiler would otherwise think this is a method declaration (other than that, the syntax is essentially equivalent)
        StateVoid wait_for_Light (this, & Navigator :: WaitForLight);
        
        StateVFF move_duration (this, & Navigator :: MoveDuration, Vector.EF, 35, 1.0); // direction, power, duration
        StateVFI move_bump (this, & Navigator :: MoveUntilBump, Vector.D, 35, D0); // direction, power, bumpID
        StateVFFI move_bump_duration (this, & Navigator :: MoveUntilBumpDuration, Vector.D, 35, 3.0, D1);  // direction, power, duration, bumpID
        
        StateFF turn_clockwise (this, & Navigator :: TurnCW, 45, 35); // degrees, power
        StateFFI turn_clockwise_bump (this, & Navigator :: TurnUntilBumpCW, 45, 35, D0); // maxDegrees, power, bumpID
        
        StateVFFII move_while_flush (this, & Navigator :: MoveWhileFlushDuration, Vector.E, 35, 4.20, F0, F1); // direction, power, duration, bump0, bump1
        StateFF set_servo_angle (this, & Navigator :: SetServoAngle, 110, 1.0); // degrees, waitTime
        
        StateF stop (this, & Navigator :: StopVehicle, 1.0); // stopTime
        

        /************************ initialize OTHER state objects ************************/
        
        StateF stop_at_the_end (this, & Navigator :: StopVehicle, 0.01); // stopTime
        StateVFF move_up_ramp_real_quick (this, & Navigator :: MoveDuration, Vector.E, 65, 1.0); // direction, power, duration
        StateFF initial_turn (this, & Navigator :: TurnCW, 135, 35); // degrees, power

        
        /************************ ADD REFERENCES of all of the state objects to the temporary state machine ************************/
        
        // note you have to start this new state machine using the GUI button (or at least that's how it's currently set up)
        TSM.Add (  & do_nothing   ); // the do nothing state should always be the first state (state [0])
        TSM.Add (  & wait_for_Light   );
        
        TSM.Add (  & initial_turn   );
        TSM.Add (  & stop   );
        TSM.Add (  & move_duration   );
        TSM.Add (  & stop   );
        TSM.Add (  & turn_clockwise   );
        
        TSM.Add (  & stop_at_the_end   ); // probably always a good idea to include a stop function at the end
        
        
        TSM.Update ();
        
        SM = StateMachine (TSM.timeWhenStateChanged, TSM.currentState); // transfer tempary non-state data into the real state machine
    }
    
    // resets the navigator object and stops the vehicle (resets state variables and that sort of thing)
    void Reset () {
        veh->Stop ();
        SM.Reset ();
    }
    // starts the navigation procedure; does this by setting state = 1
    void Start () {
        SM.Start ();
    }
    Vectors Vector;
private:
};




/************************************************************************ FUNCTION DECLERATIONS ************************************************************************/


/************************ MISC FUNCTIONS ************************/

// ...




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

// ...


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
    // initialize some constants (that are only used in the initialization method)
    const double sqrt3 = pow (3.0, 0.5);

    // initialize buttons
    char charPtr [100];
    strcpy (charPtr, "START"); // use charPtr to set the text of the button
    // initialize the start button where its parameters are: a box containing the position of the top left corner of the button, the button's width, and the button's height; a pointer to the text contained within the button; and an integer containing the button's display type (what kind of button is drawn, which coded in the button class itself)
    butt = Button (  Box ( Vector2 (30, -60), BUTT_WIDTH, BUTT_HEIGHT ), charPtr, 0  );

    strcpy (charPtr, "RESET"); // use charPtr to set the text of the button
    // initialize the reset button where its parameters are the same as the one's mentioned above
    cancelButt = Button (  Box ( Vector2 (30, -105), BUTT_WIDTH, BUTT_HEIGHT ), charPtr, 0  );

    strcpy (charPtr, "ACT"); // use charPtr to set the text of the button
    // initialize the action buttons where their parameters are the same as the one's mentioned above
    actionButt0 = Button (  Box ( Vector2 (30, -150), 45, BUTT_HEIGHT ), charPtr, 0  );
    actionButt1 = Button (  Box ( Vector2 (85, -150), 45, BUTT_HEIGHT ), charPtr, 0  );

    // initialize the course object
    course = Course (Vector2 (150, -32), .25);

    // (note that all position numbers are in inches, and relative to the top left corner of the course unless specified otherwise)
    int reverse = -1; // this variable is used to accomidate for the direction of the wheels being reversed on the actual robot
    int wheelsLength = 3; // the amount of wheels on the robot; used to determine the length of the wheels array
    int bumpsLength = 6; // the amount of bump switches on the robot; used to determine the length of the bump switch array
    float radius = 1.2; // the radius of the vehicle's wheels; used for drawing purposes and simulation calculations
    float depth = 1.6; // the depth of the vehicle's wheels; used for drawing purposes
    float hexSide = 4.25; // the value of the longest side of the vehicle's chassis
    float lowerHexRadius = (2.0*sqrt3 + 1.5*sqrt3) / 2.0; // (2*3^.5 + 1.5*3^.5) / 2.0 = 6.06217782649 / 2.0 ~= 3 // the "radius" of the lower hexagon's chassis; effectively just used to determine where the wheel's attach to the vehicles bottom
    Vector2 lowerDisp = Vector2 (0, 1.0); // displacement between the lower chassis and the upper chassis; affects where the vehicle's wheels are positioned
    // initialize the wheel objects where their parameters are: the wheel's position relative to the position of the vehicle, the forward direction of the wheel, the wheel's radius, the wheel's depth
    Wheel leftWheel = Wheel ( Vector2 (lowerDisp.x - lowerHexRadius, lowerDisp.y - lowerHexRadius), Vector2 (1 * reverse,-sqrt3 * reverse), radius, depth );
    Wheel frontWheel = Wheel ( Vector2 (lowerDisp.x, lowerDisp.y + lowerHexRadius), Vector2 (-1 * reverse, 0), radius, depth );
    Wheel rightWheel = Wheel ( Vector2 (lowerDisp.x + lowerHexRadius, lowerDisp.y - lowerHexRadius), Vector2 (1 * reverse,sqrt3 * reverse), radius, depth );

    // initialize an array of the vehicle's wheels; necessary to create a vehicle object
    Wheel wheels[] = {   leftWheel, frontWheel, rightWheel  }; // 2.75 from number friendly center

    // initialize all the bump switches where their parameters are: its position relative to the vehicle, the direction it's facing
    BumpSwitch bump0 = BumpSwitch (Vector2 (-hexSide/2.0, -hexSide), Vector2 (0, -1.0));
    BumpSwitch bump1 = BumpSwitch (Vector2 (-hexSide, 0), Vector2 (-sqrt3, 1));
    BumpSwitch bump2 = BumpSwitch (Vector2 (-hexSide/2.0, hexSide), Vector2 (-sqrt3, 1));
    BumpSwitch bump3 = BumpSwitch (Vector2 (hexSide/2.0, hexSide), Vector2 (sqrt3, 1));
    BumpSwitch bump4 = BumpSwitch (Vector2 (hexSide, 0), Vector2 (sqrt3, 1));
    BumpSwitch bump5 = BumpSwitch (Vector2 (hexSide/2.0, -hexSide), Vector2 (0, -1.0));

    // initialize an array of the vehicle's bump switches; necessary to create a vehicle object
    BumpSwitch bumps[] = {   bump0, bump1, bump2, bump3, bump4, bump5   };

    // set up the servo; define its integer min and max numbers and set its start angle
    float servoStartDegrees = 100;
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
    Vector2 chPoints[6] = {Vector2 (-hexSide/2.0, hexSide),  Vector2 (hexSide/2.0, hexSide),  Vector2 (hexSide, 0),  Vector2 (hexSide/2.0, -hexSide),  Vector2 (-hexSide/2.0, -hexSide),  Vector2 (-hexSide, 0)};
    Polygon chassis = Polygon (chPoints, 6);
    // create the vehicle object where its parameters are: its start position, start direction, center of mass, its only servo object, its only cds cell object, an array of its wheel, the length of that wheel array, an array of its bump switches, the length of that bump switch array, the radius of the lower portion of its chassis or more specifically the radius of the circle in which its wheels are tangential to (which is used in simulation calculations)
    vehicle = Vehicle (startPosition, startDirection, centerOfMass, chassis, servo, cds, wheels, wheelsLength, bumps, bumpsLength, lowerHexRadius);
    // do stuff to fix the vehicles rotation to make it better match the orientation of the real robot
    vehicle.AddRotation (255); // 180-45+120

    if (IS_SIMULATION) {
        // create a vehicle object that shows where the vehicle would by according to simulated values; this should exist in the Xcode simulation
        simulatedVehicle = Vehicle (vehicle);
        simulation = Simulation (&simulatedVehicle);
    }

    navigator = Navigator (&vehicle); // initalize the navigator object (which will be used to control vehicle navigation procedures)
    // do stuff to fix the vehicles rotation to make it better match the orientation of the real robot
    navigator.Vector.AlignVehicleVectors (75);
    Vector.AlignVehicleVectors (75);
    
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
        // update the simulated vehicle every code iteration; will adjust the simulated vehicle's position and angle
        simulation.Update ();
    }
    touchState = LCD.Touch (&touch.x, &touch.y); // get the user's touch input coordinates, and store whether or not the user is touching the screen in the variable touchState
    touch = Vector2 (touch.x, -touch.y); // standardize the touch coordinates so they're effectively in the fourth quadrant

    if (actionButt0.IsBeingPressed (touch)) { // checks if the button was pressed and updates the state of the button
        vehicle.servo.SetDegree (15); // sets the servo's angle to 15 degrees
    }
    if (actionButt1.IsBeingPressed (touch)) { // checks if the button was pressed and updates the state of the button
        vehicle.servo.SetDegree (100); // sets the servo's angle to 100 degrees
    }

    if (cancelButt.IsBeingPressed (touch)) { // checks if the button was pressed and updates the state of the button
        navigator.Reset (); // resets the navigation procedure
    }
    if (butt.IsBeingPressed (touch)) { // checks if the button was pressed and updates the state of the button
        navigator.Start (); // manually starts the navigation procedure
    }

    navigator.Update (); // perform the entire navigation procedure


    // update the LCD screen only after a fixed time interval has passed
    if (TimeNow () - timeSinceLastFrameUpdate > UPDATE_INTERVAL) { // I've heard TimeNow is bad, so the game may break after a certain amount of time has passed
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
    // LCD.WriteLine (vehicle.servo.degrees);
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

