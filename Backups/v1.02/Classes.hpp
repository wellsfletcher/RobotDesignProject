#define IS_SIMULATION 0
#if IS_SIMULATION // if this is the simulation, then this chunck of code is used. Otherwise, this code is ignored.
/*
 
 
 
 SIMULATION only stuff
 
 
 
 */






// include statements

#ifndef Classes_hpp
#define Classes_hpp
#include <stdio.h>
#endif /* Classes_hpp */

#include "GFWLCD.hpp"
#include "GFWUtility.hpp"

#include <cstdlib>                      // standard definitions
#include <iostream>                     // C++ I/O
#include <cstdio>                       // C I/O (for sprintf)






#endif
#if !IS_SIMULATION // if this is not the simulation, then this chunck of code is used. Otherwise, this code is ignored.
/*
 
 
 
 NON-simulation only stuff
 
 
 
 */






/*
#include <FEHLCD.h>
#include <FEHUtility.h>
 
#include <string.h>
*/





#endif
/*
 
 
 
 SHARED simulation & non-simulation stuff
 
 
 
 */






// include statements

#include <cmath>                        // standard definitions
using namespace std;                    // make std accessible


// universal classes

const double DEGREES_TO_RADS = M_PI / 180;

class Vector2 {
public:
    Vector2 (float x0, float y0) {
        x = x0;
        y = y0;
    }
    Vector2 () {
        x = 0;
        y = 0;
    }
    float x;
    float y;
    
    // returns the magnitude of the vector
    float getMagnitude () {
        return sqrt (  pow (x, 2.0)  +  pow (y, 2.0)  );
    }
    // returns the vector converted into a unit vector
    Vector2 getUnitVector () {
        float magnitude = getMagnitude();
        return Vector2 (x / magnitude, y / magnitude);
    }
    // rotates vector counter-clockwise 90 degrees
    void RotateLeft () {
        int tempX = x;
        x = -y;
        y = tempX;
    }
    // rotates vector clockwise 90 degrees
    void RotateRight () {
        int tempX = x;
        x = y;
        y = -tempX;
    }
    // rotates vector by given degrees counter-clockwise about the origin
    void Rotate (float degrees) {
        float rads = degrees * DEGREES_TO_RADS;
        float tempX = x;
        float tempY = y;
        x = tempX * cos (rads) - tempY * sin (rads);
        y = tempX * sin (rads) + tempY * cos (rads);
    }
    void Rotate (Vector2 direction) {
        float degrees = direction.getAngle();
        Rotate (degrees);
    }
    // gets angle formed by the vector
    float getAngle () {
        float angle = -atan (x / y) / DEGREES_TO_RADS;
        if (y <= 0) {
            angle = angle + 180;
        }
        // cout << "Angle: " << angle << endl;
        return angle;
    }
    // returns a vector equivalent for the given angle
    Vector2 DegreesToVector2 (float degrees) {
        Vector2 dir = Vector2 (1, tan (degrees * DEGREES_TO_RADS));
        return dir;
    }
};


class IntVector2 {
public:
    IntVector2 (int x0, int y0) {
        x = x0;
        y = y0;
    }
    IntVector2 () {
        x = 0;
        y = 0;
    }
    int x;
    int y;
    
    // rotates vector counter-clockwise 90 degrees
    void RotateLeft () {
        int tempX = x;
        x = -y;
        y = tempX;
    }
    // rotates vector clockwise 90 degrees
    void RotateRight () {
        int tempX = x;
        x = y;
        y = -tempX;
    }
    // returns whether the given vector equals this vector
    bool Equals (IntVector2 vect) {
        return (x == vect.x && y == vect.y);
    }
};


class Vectors {
public:
    Vectors () {
        right = Vector2 (1, 0);
        left = Vector2 (-1, 0);
        up = Vector2 (0, 1);
        down = Vector2 (0, -1);
    }
    Vector2 right;
    Vector2 left;
    Vector2 up;
    Vector2 down;
private:
};


/*
class Shape {
public:
    Shape () {
        
    }
    
    // virtual void CoolBeans () = 0;
    virtual void CoolBeans () {
        cout << "this from shape" << endl;
    }
private:
public:
    Vector2 points[8];
    char type;
};
*/


class Edge {
// class Edge {
public:
    // creates edge object, taking as its parameters: its start point, its end point, its direction
    // where its direction is used exclusively for collision and a direction of 0 is down (collides with an object traveling upwards), 1 is left, 2 is up, 3 is right
    // (the start point should generally be above and to the left of the end point in order for the edge to work properly with the physics system)
    Edge (Vector2 v0, Vector2 v1, int direction) {
        // points = {Vector2 (), Vector2()};
        points [0] = v0;
        points [1] = v1;
        dir = direction;
        switch (dir) {
            case 0:
                norm = IntVector2 (0, -1);
                break;
            case 1:
                norm = IntVector2 (-1, 0);
                break;
            case 2:
                norm = IntVector2 (0, 1);
                break;
            case 3:
                norm = IntVector2 (1, 0);
                break;
            default:
                break;
        }
    }
    Edge (Vector2 v0, Vector2 v1, IntVector2 normal) {
        points [0] = v0;
        points [1] = v1;
        norm = normal;
        if (normal.x == 0) {
            if (normal.y == 1) {
                dir = 2;
            } else if (normal.y == -1) {
                dir = 0;
            }
        } else if (normal.y == 0) {
            if (normal.x == 1) {
                dir = 3;
            } else if (normal.x == -1) {
                dir = 1;
            }
        } else {
            dir = -1;
        }
    }
    Edge () {
        
    }
    Vector2 points[2];
    int dir;
    IntVector2 norm;
};


class Box { // notably, the screen is in the fourth quadrant
public:
    // creates box object, taking four verticies of type Vector2 as its parameters
    // where v0 is top left vertex, v1 is top right vertex, v2 is bottom left vertex, v3 is bottom right vertex // should be where v0 is top left vertex, v1 is top right vertex, v2 is bottom right vertex, v3 is bottom left vertex
    Box (Vector2 v0, Vector2 v1, Vector2 v2, Vector2 v3) {
        points [0] = v0;
        points [1] = v1;
        points [2] = v2;
        points [3] = v3;
        width = points [1].x - points [0].x;
        height = points [2].y - points [0].y;
    }
    // creates box object, taking as its parameters: its top left vertex, a width, a height
    Box (Vector2 v0, float wdth, float hght) {
        width = wdth;
        height = hght;
        points [0] = v0;
        points [1] = Vector2 (v0.x + width, v0.y);
        points [2] = Vector2 (v0.x, v0.y - height); // points [2] = Vector2 (v0.x + width, v0.y - height);
        points [3] = Vector2 (v0.x + width, v0.y - height); // points [3] = Vector2 (v0.x, v0.y - height);
    }
    Box () {
        
    }
    Vector2 points[4];
    float width;
    float height;
    
    void UpdatePosition (Vector2 pos) {
        points [0] = pos;
        Updatepoints ();
    }
    void Updatepoints () {
        points [1] = Vector2 (points [0].x + width, points [0].y);
        points [2] = Vector2 (points [0].x, points [0].y - height); // points [2] = Vector2 (v0.x + width, v0.y - height);
        points [3] = Vector2 (points [0].x + width, points [0].y - height); // points [3] = Vector2 (v0.x, v0.y - height);
    }
    bool IsWithinBounds (Vector2 point) {
        if (point.x >= points [0].x  &&  point.x <= points [0].x + width  &&  point.y <= points [0].y  &&  point.y >= points [0].y - height) {
            return true;
        } else {
            return false;
        }
    }
    bool IsWithinBounds (Box box) {
        // if the box is within this.box's X bounds  &&  Y bounds
        if (  box.points [0].x <= points [1].x  &&  box.points [1].x >= points [0].x   &&   box.points [3].y <= points [0].y  &&  box.points [0].y >= points [3].y) {
            return true;
        } else {
            return false;
        }
    }
    Vector2 getCenter () {
        return Vector2 (  points [0].x + width/2.0,  points [0].y - height/2.0  );
    }
    Edge getSouthEdge () {
        Edge southEdge = Edge (points[2], points[3], 0);
        return southEdge;
    }
    Edge getWestEdge () {
        Edge westEdge = Edge (points[0], points[2], 1);
        return westEdge;
    }
    Edge getNorthEdge () {
        Edge northEdge = Edge (points[0], points[1], 2);
        return northEdge;
    }
    Edge getEastEdge () {
        Edge eastEdge = Edge (points[1], points[3], 3);
        return eastEdge;
    }
};


class Circle { // notably, the screen is in the fourth quadrant
public:
    // creates box object, taking as its parameters: its top left vertex, a width, a height
    Circle (Vector2 v0, float rds) {
        radius = rds;
        points [0] = v0;
    }
    Circle () {
        
    }
    Vector2 points[1];
    float radius;
    
    void UpdatePosition (Vector2 pos) {
        points [0] = pos;
    }
    bool IsWithinBounds (Vector2 point) {
        return false;
    }
};


class Polygon {
public:
    Polygon (Vector2 pts[], int lngth) {
        length = lngth;
        for (int k = 0; k < length; k++) {
            points [k] = pts[k];
        }
    }
    // copies an existing polygon into a new one
    Polygon (Polygon *poly) {
        length = poly->length;
        for (int k = 0; k < length; k++) {
            points [k] = Vector2 (poly->points [k].x, poly->points [k].y);
        }
    }
    Polygon () {
        
    }
    int length;
    
    void Rotate (float degrees) {
        for (int k = 0; k < length; k++) {
            points [k].Rotate (degrees);
        }
    }
    void Rotate (Vector2 direction) {
        for (int k = 0; k < length; k++) {
            points [k].Rotate (direction);
        }
    }
    void Translate (Vector2 move) {
        for (int k = 0; k < length; k++) {
            points [k] = Vector2 (points [k].x + move.x, points [k].y + move.y);
        }
    }
private:
public:
    Vector2 points [8]; // flexible array has to go at the bottom for whatever reason
};




// vehicle classes


class Wheel {
public:
    Wheel (Vector2 position, Vector2 direction, float rds, float dpth) {
        pos = position;
        dir = direction;
        radius = rds;
        depth = dpth;
        
        CreateWheel ();
    }
    Wheel (Wheel *wheel) {
        pos = Vector2 (wheel->pos.x, wheel->pos.y);
        dir = Vector2 (wheel->dir.x, wheel->dir.y);;
        radius = wheel->radius;
        depth = wheel->depth;
        
        CreateWheel ();
    }
    Wheel () {
        
    }
    // FEHMotor *motor; // a point to the motor associated with the wheel
    Vector2 pos; // position relative to the vehicle
    Vector2 dir; // the forward direction of the wheel
    // Vector2 norm; // the normal vector of the wheel (may change this to IntVector2)
    float radius;
    float depth;
    Polygon shape;
    Polygon stdShape; // the wheel's shape in standard position (with no rotation applied)
    Vector2 stdPos; // the wheel's position in standard position (with no additional rotation applied)
    Vector2 stdDir; // the wheel's direction in standard position (with no additional rotation applied)
    
    float activePercent;
    
    Vector2 lastForceApplied;
    
    // these two methods will set the percent of the motors after the "UpdateMotors" function is called in main
    void Stop () {
        activePercent = 0;
    }
    void SetPercent (float percent) {
        activePercent = percent;
    }
    void Rotate (float degrees) {
        shape.Rotate (degrees);
        pos.Rotate (degrees);
        dir.Rotate (degrees);
    }
    void SetRotation (Vector2 direction) {
        shape = Polygon (stdShape);
        shape.Rotate (direction);
        
        dir = Vector2 (stdDir.x, stdDir.y);
        pos = Vector2 (stdPos.x, stdPos.y);
        
        dir.Rotate (direction);
        pos.Rotate (direction);
    }
    void SetRotation (float degrees) {
        shape = Polygon (stdShape);
        shape.Rotate (degrees);
        
        dir = Vector2 (stdDir.x, stdDir.y);
        pos = Vector2 (stdPos.x, stdPos.y);
        
        dir.Rotate (degrees);
        pos.Rotate (degrees);
    }
private:
    void CreateWheel () {
        dir = dir.getUnitVector ();
        
        Vector2 shapePos = Vector2 (-radius, depth / 2.0);
        Box box = Box (shapePos, 2 * radius, depth);
        Vector2 points [4] = {box.points[0], box.points[1], box.points[3], box.points[2]};
        stdShape = Polygon (points, 4);
        
        stdShape.Rotate (dir.getAngle() + 90);
        stdShape.Translate (pos);
        shape = Polygon (stdShape);
        
        stdPos = Vector2 (pos.x, pos.y);
        stdDir = Vector2 (dir.x, dir.y);
        
        activePercent = 0;
    }
};


class Vehicle {
public:
    Vehicle (Vector2 position, Vector2 direction, Vector2 centerOfMass, Polygon chss, Wheel whls[], int whlsLngth, float rds) {
        pos = position;
        dir = direction.getUnitVector();
        wheelsLength = whlsLngth;
        // copy the wheels array
        for (int k = 0; k < wheelsLength; k++) {
            wheels [k] = Wheel (whls [k]);
        }
        chassis = Polygon (chss);
        stdChassis = Polygon (chss);
        CM = centerOfMass; // center of mass should usually be Vector2 (0, 0) by default
        radius = rds;
    }
    Vehicle (Vehicle *veh) {
        pos = Vector2 (veh->pos.x, veh->pos.y);
        dir = Vector2 (veh->dir.x, veh->dir.y);
        wheelsLength = veh->wheelsLength;
        // copy the wheels array
        for (int k = 0; k < wheelsLength; k++) {
            // wheels [k] = whls [k];
            wheels [k] = Wheel (veh->wheels [k]);
        }
        chassis = Polygon (veh->chassis);
        stdChassis = Polygon (veh->stdChassis);
        CM = Vector2 (veh->CM.x, veh->CM.y);
        radius = veh->radius;
    }
    Vehicle () {
        
    }
    // general variables
    Vector2 pos; // the position of the vehicle
    Vector2 dir; // the forward direction of the vehicle
    int wheelsLength;
    Vector2 CM; // center of mass
    float radius;
    Vector2 lastForceApplied; // for reference and drawing
    
    // variables primarily used for simulation
    Vector2 vel; // the velocity of the vehicle
    float angVel; // the angular velocity of the vehicle in degrees per second
    
    
    /*********************** general functions *************************/
    
    // ...
    
    
    /*********************** navigation / movement functions *************************/
    
    void Move (Vector2 motorPercent2) {
        float sqrt3 = sqrt (3.0);
        float w0 = -0.5 * motorPercent2.x  -  (sqrt3/2.0) * motorPercent2.y;
        float w1 = motorPercent2.x;
        float w2 = -0.5 * motorPercent2.x  +  (sqrt3/2.0) * motorPercent2.y;
        
        wheels [0].SetPercent (w0);
        wheels [1].SetPercent (w1);
        wheels [2].SetPercent (w2);
    }
    // sets wheel speeds using a given unit vector direction and its magnitude (where its magnitude corresponds to the motor's power percent)
    void Move (Vector2 direction, float magnitude) {
        float sqrt3 = sqrt (3.0);
        float w0 = 0.5 * direction.x  -  (sqrt3/2.0) * direction.y;
        float w1 = -direction.x;
        float w2 = 0.5 * direction.x  +  (sqrt3/2.0) * direction.y;
        
        w0 *= magnitude;
        w1 *= magnitude;
        w2 *= magnitude;
        
        wheels [0].SetPercent (w0);
        wheels [1].SetPercent (w1);
        wheels [2].SetPercent (w2);
    }
    void MoveForward (float motorPercent) {
        Move (Vector2 (0, motorPercent));
    }
    void Stop () {
        wheels [0].Stop ();
        wheels [1].Stop ();
        wheels [2].Stop ();
    }
    
    
    /*********************** all these functions are basically for simulation purposes *************************/
    
    void ApplyAcceleration (Vector2 accel) {
        vel = Vector2 (vel.x + accel.x, vel.y + accel.y);
    }
    // updates the vehicles position by adding its velocity to its position
    void UpdatePosition () {
        pos = Vector2 (pos.x + vel.x, pos.y + vel.y);
    }
    void ApplyAngularAcceleration (float angAccel) {
        angVel = angVel + angAccel;
    }
    void UpdateRotation () {
        AddRotation (angVel);
    }
    void AddRotation (float degrees) {
        chassis.Rotate (degrees);
        dir.Rotate (degrees);
        
        for (int k = 0; k < wheelsLength; k++) {
            wheels [k].Rotate (degrees);
        }
    }
    void SetRotation (Vector2 direction) {
        dir = direction;
        chassis = Polygon (stdChassis);
        chassis.Rotate (direction);
        
        for (int k = 0; k < wheelsLength; k++) {
            wheels [k].SetRotation (direction);
        }
    }
    void SetRotation (float degrees) {
        dir = dir.DegreesToVector2 (degrees); // theta = arctan (y / 1) --> tan (theta) = y / 1
        chassis = Polygon (stdChassis);
        chassis.Rotate (degrees);
        
        for (int k = 0; k < wheelsLength; k++) {
            wheels [k].SetRotation (degrees);
        }
    }
    void SetPosition (Vector2 position) {
        pos = position;
    }
private:
    // Vector2 center; // geometric center of vehicle (may be redundant)
public:
    Polygon stdChassis; // the vehicle's chassis shape in standard position (with no rotation applied)
    Polygon chassis;
    Wheel wheels [8]; // flexible array has to go at the bottom for whatever reason
};




// graphics classes


class Button {
public:
    // creates a box object, taking as its parameters a box object representing its boundary, a character pointer to an array indicating the text
    // to be displayed on the object, and an an integer representing the type of button it is, where 0 is default menu button and 5 is arrow pad button
    Button (Box bounds, char txt[], int buttonType) {
        box = bounds;
        strcpy (text, txt);
        type = buttonType;
        BUTT_MARGIN = 5;
        pressStartedWithinBound = false;
    }
    Button (Box bounds) {
        box = bounds;
        type = 0;
        strcpy (text, "");
        BUTT_MARGIN = 5;
    }
    Button () {
        
    }
    Box box;
    int type;
    char text[100];
    bool isPressed; // button's state of being pressed or unpressed
    bool noPressLastTime;
    bool pressStartedWithinBound;
    int BUTT_MARGIN;
    // these may need to be moved / initialized elswhere in order to work on the proteus
    static const int dec = 5; // amount pixels to shrink the button by when its pressed
    static const int TEXT_HEIGHT = 10;
    
    // updates the text of the button
    void UpdateText (char txt[]) {
        strcpy (text, txt);
    }
    // returns whether the button had been pressed and updates the pressed-or-unpressed state of the button
    bool WasPressed (Vector2 touchPos, bool touchStatus) {
        bool result = false;
        if (touchStatus == false) { // if the screen is not currently touched
            if (isPressed == true && pressStartedWithinBound) { // check if the button had been touched the last time the method was called and that the initial touch was within the bounds of the button
                result = true; // if it was, then return that the button was pressed
                // *****
                //       this bit could not be working because LCD.Touch only updates the returned mouse position when the screen is first touched
                //       I can fix that in this 'emulation' but I don't know what the real behavior of LCD.Touch is on the actual proteus
                //       so if I fix this now it may cause problems with button input in the future
                // *****
                isPressed = false; // reset the button so no weird behaviors happen
                pressStartedWithinBound = false;
                noPressLastTime = true;
            } else { // if the screen had never been touched or if when the screen was no longer being touched the touch input was not within the bounds of the button
                result = false;
                noPressLastTime = true; // take note that there was no press during this iteration of the method's call
                pressStartedWithinBound = false;
            }
        } else { // if the screen is touched
            isPressed = IsWithinBounds (touchPos); // check if the touch input was in the bounds of the button
            // pressStartedWithinBound = true; // un-comment this line to make it so that it doesn't matter if the touch input initially started within the bounds of the button
            if (noPressLastTime && isPressed) { // if this is the first time there is a touch input on the screen and the input is within the bounds of the button
                pressStartedWithinBound = true; // take note of it
            } if (pressStartedWithinBound == false) { // if the initial press was not within the bounds of the button
                isPressed = false; // make it be so that the button is not pressed
            }
            result = false;
            noPressLastTime = false;
        }
        return result;
    }
    // returns whether the button is being pressed and updates the pressed-or-unpressed state of the button
    bool IsBeingPressed (Vector2 touchPos) {
        isPressed = IsWithinBounds (touchPos);
        return isPressed;
    }
    /*
    bool IsBeingPressed (Vector2 touchPos, bool touchStatus) {
        isPressed = IsWithinBounds (touchPos);
        return isPressed;
    }
    */
    // returns true if the touch position is within the position of the button boundaries and false otherwise
    bool IsWithinBounds (Vector2 touchPos) {
        if (touchPos.x > box.points [0].x  &&  touchPos.x < box.points [0].x + box.width  &&  touchPos.y < box.points [0].y  &&  touchPos.y > box.points [0].y - box.height) { // may have to adjust this based on where the origin is set
            return true;
        } else {
            return false;
        }
    }
    // draws pressed or unpressed button depending on the button's state
    void DrawButton () {
        if (isPressed) { // if the button is pressed
            DrawPressedButton ();
        } else {
            DrawUnpressedButton ();
        }
    }
private:
    // these may need to be moved / initialized elswhere in order to work on the proteus
    static const int ABYSS = 0x1a1a1a;
    static const int WALLCOLOR = ABYSS;
    static const int BUTTHIGHLIGHT = INDIANRED;
    
    // draws unpressed button
    void DrawUnpressedButton () {
        switch (type) {
            case 5:
                LCD.SetFontColor (WHITE);
                LCD.WriteAt ( text, (int)(box.points[0].x+BUTT_MARGIN), -(int)(box.points[0].y - box.height / 2.0 + TEXT_HEIGHT / 2.0) );
                LCD.DrawRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y), box.width, box.height );
                break;
            case 6:
                LCD.SetFontColor (ABYSS);
                LCD.SetBackgroundColor (ABYSS);
                LCD.FillRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y), box.width, box.height );
                LCD.SetFontColor (WHITE);
                LCD.WriteAt ( text, (int)(box.points[0].x+BUTT_MARGIN), -(int)(box.points[0].y - box.height / 2.0 + TEXT_HEIGHT / 2.0) );
                LCD.DrawRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y), box.width, box.height );
                break;
            case 0:
            default:
                // LCD.FillRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y), box.width, box.height);
                // effectively erase the button before drawing the un-pressed version
                LCD.SetFontColor (WALLCOLOR);
                LCD.FillRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y) - 1, box.width + 1, box.height + 1 );
                // draw the actual button
                LCD.SetFontColor (WHITE);
                LCD.WriteAt ( text, (int)(box.points[0].x+BUTT_MARGIN), -(int)(box.points[0].y - box.height / 2.0 + TEXT_HEIGHT / 2.0) );
                LCD.DrawRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y), box.width, box.height );
                break;
        }
    }
    // draws pressed version of button
    void DrawPressedButton () {
        switch (type) {
            case 5:
                LCD.SetFontColor (RED);
                LCD.WriteAt ( text, (int)(box.points[0].x+BUTT_MARGIN), -(int)(box.points[0].y - box.height / 2.0 + TEXT_HEIGHT / 2.0) );
                // LCD.DrawRectangle ( (int)(box.points[0].x + dec / 2.0), -(int)(box.points[0].y - dec / 2.0), box.width - dec, box.height - dec );
                LCD.DrawRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y), box.width, box.height );
                break;
            case 6:
                LCD.SetFontColor (RED);
                LCD.WriteAt ( text, (int)(box.points[0].x+BUTT_MARGIN), -(int)(box.points[0].y - box.height / 2.0 + TEXT_HEIGHT / 2.0) );
                break;
            case 0:
            default:
                // effectively erase the button before drawing the pressed version
                LCD.SetFontColor (WALLCOLOR);
                LCD.FillRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y) - 1, box.width + 1, box.height + 1 ); // erase button
                LCD.SetFontColor (BUTTHIGHLIGHT);
                LCD.SetBackgroundColor (BUTTHIGHLIGHT);
                LCD.FillRectangle ( (int)(box.points[0].x + dec / 2.0), -(int)(box.points[0].y - dec / 2.0), box.width - dec, box.height - dec );
                LCD.SetFontColor (WHITE);
                LCD.WriteAt ( text, (int)(box.points[0].x+BUTT_MARGIN), -(int)(box.points[0].y - box.height / 2.0 + TEXT_HEIGHT / 2.0));
                LCD.DrawRectangle ( (int)(box.points[0].x + dec / 2.0), -(int)(box.points[0].y - dec / 2.0), box.width - dec, box.height - dec );
                LCD.SetBackgroundColor (BLACK);
                break;
        }
    }
};
