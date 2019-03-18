#define IS_SIMULATION 1
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







#include <FEHLCD.h>
#include <FEHUtility.h>

#include <string.h>






#endif
/*



 SHARED simulation & non-simulation stuff



 */






// include statements

#include <cmath>                        // standard definitions
using namespace std;                    // make std accessible

//float CF0 = 1.0; // correction factor for wheel 0 (the one in motor port 0)
//float CF1 = 1.0; // correction factor for wheel 1 (the one in motor port 1)
//float CF2 = 1.0; // correction factor for wheel 2 (the one in motor port 2)

// universal classes

#define M_PI 3.141592653589
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
        float magnitude = getMagnitude ();
        Vector2 unitVector;
        
        if (magnitude == 0) {
            unitVector = Vector2 (0, 0);
        } else {
            unitVector = Vector2 (x / magnitude, y / magnitude);
        }
        
        return unitVector;
    }
    // clamps the magnitude of the vector to maxLength
    void ClampMagnitude (float maxLength) {
        Vector2 unitVect = getUnitVector ();
        x = unitVect.x * maxLength;
        y = unitVect.y * maxLength;
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
    // gets angle formed by the vector, but ensures that the returned angle is between 0 and 360
    float getCappedAngle () {
        float cappedAngle = getAngle ();
        CapDegrees (cappedAngle);
        return cappedAngle;
    }
    // returns the degrees capped at 360 and 0
    const static float CapDegrees (float degrees) {
        if (degrees >= 360) {
            degrees -= 360;
        } else if (degrees < 0) {
            degrees += 360;
        }
        return degrees;
    }
    // returns a vector equivalent for the given angle
    static Vector2 DegreesToVector2 (float degrees) {
        Vector2 dir = Vector2 (1, tan (degrees * DEGREES_TO_RADS));
        return dir;
    }
    // gets Cross Product between this and other (this x other)
    float CalculateCrossProduct (Vector2 other) {
        float result = x * other.y  -  y * other.x;
        return result;
    }
    // calculates dot product
    const static float Dot (Vector2 first, Vector2 second) {
        float result = first.x * second.x  +  first.y * second.y;
        return result;
    }
    // gets projection of the given vector onto this (in direction of this; magnitude -> other)
    const static Vector2 Proj (Vector2 dir, Vector2 vect) {
        // norm #this * #this
        float bottom = dir.getMagnitude (); // or dir dot dir
        // dot #this and other
        float dot = Dot (dir, vect);
        // scaler = dot / norm
        float scalar = dot / bottom;
        // this * scalar
        Vector2 proj = Vector2 (dir.x * scalar, dir.y * scalar);
        
        return proj;
    }
    // gets "scal" of the given vector onto this (in direction of this; magnitude -> other)
    const static float Scal (Vector2 dir, Vector2 vect) {
        // norm #this * #this
        float bottom = dir.getMagnitude (); // or dir dot dir
        // dot #this and other
        float dot = Dot (dir, vect);
        // scaler = dot / norm
        float scalar = dot / bottom;
        
        return scalar;
    }
    // gets the slope from the first vector to the second vector in vector form
    const static Vector2 SlopeVect (Vector2 first, Vector2 second) {
        Vector2 slope = Vector2 (second.x - first.x, second.y - first.y);
        
        return slope;
    }
    // calculates distance between two points
    const static float Distance (Vector2 start, Vector2 end) {
        return sqrt (  pow (end.x - start.x, 2.0)  +  pow (end.y - start.y, 2.0)  );
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

        const double sqrt3 = sqrt (3.0);

        Bb = Vector2 (-sqrt3, 1).getUnitVector ();
        C = Vector2 (0, 1).getUnitVector ();
        D = Vector2 (sqrt3, 1).getUnitVector ();
        E = Vector2 (sqrt3, -1).getUnitVector ();
        F = Vector2 (0, -1).getUnitVector ();
        Aa = Vector2 (-sqrt3, -1).getUnitVector ();

        OldD = Vector2 (sqrt3, 1);
        OldE = Vector2 (sqrt3, -1);

        AB = getAverageUnitVector2 (Aa, Bb);
        BC = getAverageUnitVector2 (Bb, C);
        CD = getAverageUnitVector2 (C, D);
        DE = getAverageUnitVector2 (D, E);
        EF = getAverageUnitVector2 (E, F);
        FA = getAverageUnitVector2 (F, Aa);

        OldDE = getAverageUnitVector2 (OldD, OldE);
    }
    Vector2 right;
    Vector2 left;
    Vector2 up;
    Vector2 down;

    Vector2 Aa;
    Vector2 Bb;
    Vector2 C;
    Vector2 D;
    Vector2 E;
    Vector2 F;

    Vector2 AB;
    Vector2 BC;
    Vector2 CD;
    Vector2 DE;
    Vector2 EF;
    Vector2 FA;

    Vector2 OldD;
    Vector2 OldE;
    Vector2 OldDE;

    void AlignVehicleVectors (float degrees) {
        float degreesForAlignment = degrees;

        Aa.Rotate (degreesForAlignment);
        Bb.Rotate (degreesForAlignment);
        C.Rotate (degreesForAlignment);
        D.Rotate (degreesForAlignment);
        E.Rotate (degreesForAlignment);
        F.Rotate (degreesForAlignment);

        AB.Rotate (degreesForAlignment);
        BC.Rotate (degreesForAlignment);
        CD.Rotate (degreesForAlignment);
        DE.Rotate (degreesForAlignment);
        EF.Rotate (degreesForAlignment);
        FA.Rotate (degreesForAlignment);

        OldD.Rotate (degreesForAlignment);
        OldE.Rotate (degreesForAlignment);
        OldDE.Rotate (degreesForAlignment);
    }
    Vector2 getAverageUnitVector2 (Vector2 a, Vector2 b) {
        Vector2 average = Vector2 (a.x + b.x, a.y + b.y);
        average = average.getUnitVector ();
        return average;
    }

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
        norm = DirToIntVector2 (direction);
        normal = Vector2 (norm.x, norm.y).getUnitVector ();
    }
    Edge (Vector2 v0, Vector2 v1, IntVector2 norm0) {
        points [0] = v0;
        points [1] = v1;
        norm = norm0;
        dir = IntVector2ToDir (norm0);
        normal = Vector2 (norm.x, norm.y).getUnitVector ();
    }
    // creates an edge object, and calculates the normal vector automatically
    Edge (Vector2 v0, Vector2 v1) {
        points [0] = v0;
        points [1] = v1;
        norm = IntVector2 (0, 0);
        dir = 0;
        normal = getNormal ().getUnitVector ();
    }
    Edge () {

    }
    Vector2 points [2];
    int dir;
    IntVector2 norm; // legacy normal vector
    Vector2 normal; // the real normal vector
    
    // flips the two points that make up the edge; also updates the edges normals
    void FlipPoints () {
        Vector2 tempPoint = points [0];
        points [0] = points [1];
        points [1] = tempPoint;
        normal = getNormal ().getUnitVector ();
    }
    // calculates and returns the normal vector of the edge
    Vector2 getNormal () {
        // Vector2 tempNormal = Vector2 (0, 0);
        Vector2 tempNormal = Vector2 (points [1].x - points [0].x, points [1].y - points [0].y);
        tempNormal.RotateLeft ();
        return tempNormal;
    }
    const static IntVector2 DirToIntVector2 (int tempDir) {
        IntVector2 tempNorm;
        switch (tempDir) {
            case 0:
                tempNorm = IntVector2 (0, -1);
                break;
            case 1:
                tempNorm = IntVector2 (-1, 0);
                break;
            case 2:
                tempNorm = IntVector2 (0, 1);
                break;
            case 3:
                tempNorm = IntVector2 (1, 0);
                break;
            default:
                break;
        }
        return tempNorm;
    }
    const static int IntVector2ToDir (IntVector2 tempNorm) {
        int tempDir = 0;
        if (tempNorm.x == 0) {
            if (tempNorm.y == 1) {
                tempDir = 2;
            } else if (tempNorm.y == -1) {
                tempDir = 0;
            }
        } else if (tempNorm.y == 0) {
            if (tempNorm.x == 1) {
                tempDir = 3;
            } else if (tempNorm.x == -1) {
                tempDir = 1;
            }
        } else {
            tempDir = -1;
        }
        return tempDir;
    }
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
    
    /*
    Edge getInvertedSouthEdge () {
        Edge southEdge = Edge (points[2], points[3], 2);
        return southEdge;
    }
    Edge getInvertedWestEdge () {
        Edge westEdge = Edge (points[0], points[2], 3);
        return westEdge;
    }
    Edge getInvertedNorthEdge () {
        Edge northEdge = Edge (points[0], points[1], 0);
        return northEdge;
    }
    Edge getInvertedEastEdge () {
        Edge eastEdge = Edge (points[1], points[3], 1);
        return eastEdge;
    }
    */
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
    // converts the given shape to a polygon
    const static Polygon ConvertToPolygon (Box shape) {
        Vector2 polyPoints [4] = {   shape.points [0],  shape.points [1],  shape.points [3],  shape.points [2]  };
        Polygon poly = Polygon (polyPoints, 4);
        return poly;
    }
    // converts the given shape to a polygon
    const static Polygon ConvertToPolygon (Edge shape) {
        Vector2 polyPoints [2] = {   shape.points [0],  shape.points [1]  }; // hmmm... not really a polygon
        Polygon poly = Polygon (polyPoints, 2);
        return poly;
    }
    // converts the given shape to an inverted polygon
    const static Polygon ConvertToFlippedPolygon (Box shape) {
        Vector2 polyPoints [4] = {   shape.points [0],  shape.points [2],  shape.points [3],  shape.points [1]  };
        Polygon poly = Polygon (polyPoints, 4);
        return poly;
    }
private:
public:
    Vector2 points [8]; // flexible array has to go at the bottom for whatever reason
};




// vehicle classes

class CDS {
public:
    CDS (Vector2 position, Vector2 direction) {
        pos = position;
        dir = direction;

        CreateCDS ();
    }
    CDS (CDS *cds) {
        pos = Vector2 (cds->pos.x, cds->pos.y);
        dir = Vector2 (cds->dir.x, cds->dir.y);;

        CreateCDS ();
    }
    CDS () {

    }
    Vector2 pos; // position relative to the vehicle
    Vector2 dir; // the forward direction (normal) of the bump switch
    Vector2 stdPos; // the bump's position in standard position (with no additional rotation applied)
    Vector2 stdDir; // the bump's direction in standard position (with no additional rotation applied)

    float value;

    // for use with red color filter
    bool isBlueLight () {
        float lowerBound = 0.55;
        float upperBound = 1.55; // should have been 1.7 ...s
        bool result = false;

        if (value >= lowerBound && value < upperBound) {
            result = true;
        }

        return result;
    }
    bool isBlueLight (float testValue) { // note this is not for testing the current value of the CDS cell
        float lowerBound = 0.55;
        float upperBound = 1.7; // 1.55
        bool result = false;

        if (testValue >= lowerBound && testValue < upperBound) {
            result = true;
        }

        return result;
    }

    // for use with red color filter
    bool isRedLight () {
        float lowerBound = 0;
        float upperBound = 0.55;
        bool result = false;

        if (value >= lowerBound && value < upperBound) {
            result = true;
        }

        return result;
    }
    bool isRedLight (float testValue) {  // note this is not for testing the current value of the CDS cell
        float lowerBound = 0;
        float upperBound = 0.55;
        bool result = false;

        if (testValue >= lowerBound && testValue < upperBound) {
            result = true;
        }

        return result;
    }

    // for use with no color filter
    bool isLight () {
        float lowerBound = 0;
        float upperBound = 1.0;
        bool result = false;

        if (value >= lowerBound && value < upperBound) {
            result = true;
        }

        return result;
    }

    float Value () {
        return value;
    }
    // don't use this
    void SetValue (float newValue) {
        value = newValue;
    }
    void Rotate (float degrees) {
        pos.Rotate (degrees);
        dir.Rotate (degrees);
    }
    void SetRotation (Vector2 direction) {
        dir = Vector2 (stdDir.x, stdDir.y);
        pos = Vector2 (stdPos.x, stdPos.y);

        dir.Rotate (direction);
        pos.Rotate (direction);
    }
    void SetRotation (float degrees) {
        dir = Vector2 (stdDir.x, stdDir.y);
        pos = Vector2 (stdPos.x, stdPos.y);

        dir.Rotate (degrees);
        pos.Rotate (degrees);
    }
private:
    void CreateCDS () {
        dir = dir.getUnitVector ();

        stdPos = Vector2 (pos.x, pos.y);
        stdDir = Vector2 (dir.x, dir.y);

        value = true;
    }
};


class Servo {
public:
    Servo (Vector2 position, Vector2 direction, float startDegrees) {
        pos = position;
        dir = direction;
        degrees = startDegrees;

        CreateServo ();
    }
    Servo (Servo *serv) {
        pos = Vector2 (serv->pos.x, serv->pos.y);
        dir = Vector2 (serv->dir.x, serv->dir.y);
        degrees = serv->degrees;

        CreateServo ();
    }
    Servo () {

    }
    Vector2 pos; // position relative to the vehicle
    Vector2 dir; // the forward direction (normal) of the bump switch
    Vector2 stdPos; // the bump's position in standard position (with no additional rotation applied)
    Vector2 stdDir; // the bump's direction in standard position (with no additional rotation applied)

    // bool minIsCalibrated; // eh this doesn't actually need to exist
    // bool maxIsCalibrated;
    float degrees;
    int min;
    int max;
    bool off;

    void SetDegree (float dgrs) {
        degrees = dgrs;
    }
    /*
    void SetMin (int minValue) {
        min = minValue;
        // minIsCalibrated = true;
    }
    void SetMax (int maxValue) {
        max = maxValue;
        // maxIsCalibrated = true;
    }
    */
    void Off () {
        off = true;
    }

    void Rotate (float degrees) {
        pos.Rotate (degrees);
        dir.Rotate (degrees);
    }
    void SetRotation (Vector2 direction) {
        dir = Vector2 (stdDir.x, stdDir.y);
        pos = Vector2 (stdPos.x, stdPos.y);

        dir.Rotate (direction);
        pos.Rotate (direction);
    }
    void SetRotation (float degrees) {
        dir = Vector2 (stdDir.x, stdDir.y);
        pos = Vector2 (stdPos.x, stdPos.y);

        dir.Rotate (degrees);
        pos.Rotate (degrees);
    }
private:
    void CreateServo () {
        dir = dir.getUnitVector ();

        stdPos = Vector2 (pos.x, pos.y);
        stdDir = Vector2 (dir.x, dir.y);

        min = 500;
        max = 2500;
        off = false;
    }
};


class BumpSwitch {
public:
    BumpSwitch (Vector2 position, Vector2 direction) {
        pos = position;
        dir = direction;

        CreateBumpSwitch ();
    }
    BumpSwitch (BumpSwitch *bump) {
        pos = Vector2 (bump->pos.x, bump->pos.y);
        dir = Vector2 (bump->dir.x, bump->dir.y);;

        CreateBumpSwitch ();
    }
    BumpSwitch () {

    }
    Vector2 pos; // position relative to the vehicle
    Vector2 dir; // the forward direction (normal) of the bump switch
    Vector2 stdPos; // the bump's position in standard position (with no additional rotation applied)
    Vector2 stdDir; // the bump's direction in standard position (with no additional rotation applied)
    Vector2 startPos; // used for debugging the simulation; should delete in the future
    Vector2 endPos; // used for debugging the simulation; should delete in the future


    bool value;

    // these two methods will set the value of the bumps after the "UpdateBumps" function is called in main
    bool Value () {
        return value;
    }
    // don't use this
    void SetValue (bool newValue) {
        value = newValue;
    }
    void Rotate (float degrees) {
        pos.Rotate (degrees);
        dir.Rotate (degrees);
    }
    void SetRotation (Vector2 direction) {
        dir = Vector2 (stdDir.x, stdDir.y);
        pos = Vector2 (stdPos.x, stdPos.y);

        dir.Rotate (direction);
        pos.Rotate (direction);
    }
    void SetRotation (float degrees) {
        dir = Vector2 (stdDir.x, stdDir.y);
        pos = Vector2 (stdPos.x, stdPos.y);

        dir.Rotate (degrees);
        pos.Rotate (degrees);
    }
private:
    void CreateBumpSwitch () {
        dir = dir.getUnitVector ();

        stdPos = Vector2 (pos.x, pos.y);
        stdDir = Vector2 (dir.x, dir.y);

        value = true;
    }
};


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
    Vehicle (Vector2 position, Vector2 direction, Vector2 centerOfMass, Polygon chss, Servo servoMotor, CDS cdsCell, Wheel whls[], int whlsLngth, BumpSwitch bmps[], int bmpsLngth, float rds) {
        pos = position;
        dir = direction.getUnitVector();
        // copy CDS cell object
        cds = CDS (cdsCell);
        // copy servo object
        servo = Servo (servoMotor);
        // copy the wheels array
        wheelsLength = whlsLngth;
        for (int k = 0; k < wheelsLength; k++) {
            wheels [k] = Wheel (whls [k]);
        }
        chassis = Polygon (chss);
        stdChassis = Polygon (chss);
        CM = centerOfMass; // center of mass should usually be Vector2 (0, 0) by default
        radius = rds;

        bumpsLength = bmpsLngth;
        // copy the bumps array
        for (int k = 0; k < bumpsLength; k++) {
            bumps [k] = BumpSwitch (bmps [k]);
        }
        
        CF0 = 1.0; // correction factor for wheel 0 (the one in motor port 0)
        CF1 = 1.0; // correction factor for wheel 1 (the one in motor port 1)
        CF2 = 1.0; // correction factor for wheel 2 (the one in motor port 2)
        
        color = BLACK;
        isSimulated = false;
    }
    Vehicle (Vehicle *veh) {
        pos = Vector2 (veh->pos.x, veh->pos.y);
        dir = Vector2 (veh->dir.x, veh->dir.y);
        // copy CDS cell object
        cds = CDS (veh->cds);
        // copy servo object
        servo = Servo (veh->servo);
        // copy the wheels array
        wheelsLength = veh->wheelsLength;
        for (int k = 0; k < wheelsLength; k++) {
            // wheels [k] = whls [k];
            wheels [k] = Wheel (veh->wheels [k]);
        }
        chassis = Polygon (veh->chassis);
        stdChassis = Polygon (veh->stdChassis);
        CM = Vector2 (veh->CM.x, veh->CM.y);
        radius = veh->radius;

        bumpsLength = veh->bumpsLength;
        // copy the bumps array
        for (int k = 0; k < bumpsLength; k++) {
            bumps [k] = BumpSwitch (veh->bumps [k]);
        }
        CF0 = veh->CF0; // correction factor for wheel 0 (the one in motor port 0)
        CF1 = veh->CF1; // correction factor for wheel 1 (the one in motor port 1)
        CF2 = veh->CF2; // correction factor for wheel 2 (the one in motor port 2)
        
        color = BLACK;
        isSimulated = veh->isSimulated;
    }
    Vehicle () {

    }
    // general variables
    Vector2 pos; // the position of the vehicle
    Vector2 dir; // the forward direction of the vehicle
    int wheelsLength;
    int bumpsLength;
    Vector2 CM; // center of mass
    float radius;
    Vector2 lastForceApplied; // for reference and drawing

    // variables primarily used for simulation
    Vector2 vel; // the velocity of the vehicle
    float angVel; // the angular velocity of the vehicle in degrees per second
    
    // graphics
    int color;
    bool isSimulated; // whether the vehicle is a simulated vehicle or not


    /*********************** general functions *************************/

    // ...


    /*********************** navigation / movement functions *************************/

    float CF0; // correction factor for wheel 0 (the one in motor port 0)
    float CF1; // correction factor for wheel 1 (the one in motor port 1)
    float CF2; // correction factor for wheel 2 (the one in motor port 2)

    // this is the one that is used
    // turns counterclockwise; make sure the motor percent doesn't exceed 100; the vehicle's linear speed remains constant
    void Turn (float motorPercent) {
        float w0 = wheels [0].activePercent + motorPercent;
        float w1 = wheels [1].activePercent + motorPercent;
        float w2 = wheels [2].activePercent + motorPercent;

        // multiply wheels by wheel correction factor
        w0 = w0 * CF0;
        w1 = w1 * CF1;
        w2 = w2 * CF2;

        wheels [0].SetPercent (w0);
        wheels [1].SetPercent (w1);
        wheels [2].SetPercent (w2);
    }
    // turns counterclockwise; the vehicle's linear speed does not remain constant, but the net motor output remains constant (I haven't finished implementing this method correctly yet)
    void TurnNeutral (float motorPercent) { // not used
        float w0 = wheels [0].activePercent;
        float w1 = wheels [1].activePercent;
        float w2 = wheels [2].activePercent;

        w0 *= 1 - motorPercent/100.0;
        w1 *= 1 - motorPercent/100.0;
        w2 *= 1 - motorPercent/100.0;

        w0 += motorPercent;
        w1 += motorPercent;
        w2 += motorPercent;

        wheels [0].SetPercent (w0);
        wheels [1].SetPercent (w1);
        wheels [2].SetPercent (w2);
    }
    void Move (Vector2 motorPercent2) { // not used
        float sqrt3 = sqrt (3.0);
        float w0 = -0.5 * motorPercent2.x  -  (sqrt3/2.0) * motorPercent2.y;
        float w1 = motorPercent2.x;
        float w2 = -0.5 * motorPercent2.x  +  (sqrt3/2.0) * motorPercent2.y;

        // multiply wheels by wheel correction factor
        w0 *= CF0;
        w1 *= CF1;
        w2 *= CF2;

        wheels [0].SetPercent (w0);
        wheels [1].SetPercent (w1);
        wheels [2].SetPercent (w2);
    }
    // this is the one that is used
    // sets wheel speeds using a given unit vector direction and its magnitude (where its magnitude corresponds to the motor's power percent)
    void Move (Vector2 direction, float magnitude) {
        // cout << "Mag: " << magnitude << endl;
        // direction = direction.getUnitVector();
        
        float sqrt3 = sqrt (3.0);
        float w0 = 0.5 * direction.x  -  (sqrt3/2.0) * direction.y;
        float w1 = -direction.x;
        float w2 = 0.5 * direction.x  +  (sqrt3/2.0) * direction.y;

        w0 *= magnitude;
        w1 *= magnitude;
        w2 *= magnitude;

        // multiply wheels by wheel correction factor
        w0 *= CF0;
        w1 *= CF1;
        w2 *= CF2;

        /*
        cout << "1: " << w0 << endl;
        cout << "2: " << w1 << endl;
        cout << "3: " << w2 << endl;
        */
        
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
        for (int k = 0; k < bumpsLength; k++) {
            bumps [k].Rotate (degrees);
        }
    }
    void SetRotation (Vector2 direction) {
        dir = direction;
        chassis = Polygon (stdChassis);
        chassis.Rotate (direction);

        for (int k = 0; k < wheelsLength; k++) {
            wheels [k].SetRotation (direction);
        }
        for (int k = 0; k < bumpsLength; k++) {
            bumps [k].SetRotation (direction);
        }
    }
    void SetRotation (float degrees) {
        dir = dir.DegreesToVector2 (degrees); // theta = arctan (y / 1) --> tan (theta) = y / 1
        chassis = Polygon (stdChassis);
        chassis.Rotate (degrees);

        for (int k = 0; k < wheelsLength; k++) {
            wheels [k].SetRotation (degrees);
        }
        for (int k = 0; k < bumpsLength; k++) {
            bumps [k].SetRotation (degrees);
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
    Servo servo;
    CDS cds;
    Wheel wheels [8]; // flexible array has to go at the bottom for whatever reason
    BumpSwitch bumps [6];
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
                LCD.SetBackgroundColor (WALLCOLOR);
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


class StateIndicator {
public:
    // creates a box object, taking as its parameters a box object representing its boundary, a character pointer to an array indicating the text
    // to be displayed on the object, // and an an integer representing the type of indicator it is
    StateIndicator (Box bounds, char txt[]) {
        box = bounds;
        strcpy (text, txt);
        type = 0;
        BUTT_MARGIN = 5;
        
        backgroundColor = ABYSS;
        borderColor = WHITE;
        textColor = WHITE;
    }
    StateIndicator () {
        
    }
    Box box;
    int type;
    char text[100];
    int BUTT_MARGIN;
    static const int TEXT_HEIGHT = 10;
    int backgroundColor;
    int borderColor;
    int textColor;
    
    // updates the text of the indicator
    void UpdateText (char txt[]) {
        strcpy (text, txt);
    }
    // updates the color of the indicator, given a background color, border color, and text color
    void UpdateColors (int background, int border, int textC) {
        backgroundColor = background;
        borderColor = border;
        textColor = textC;
    }
    // updates the color of the indicator, given a background color and border color
    void UpdateColors (int background, int border) {
        backgroundColor = background;
        borderColor = border;
    }
    // updates the color of the indicator, given a background color
    void UpdateColors (int background) {
        backgroundColor = background;
    }
    // draws the indicator
    void Draw () {
        DrawIndicator (); // this method is kinda redundant
    }
private:
    static const int ABYSS = 0x1a1a1a;
    
    // actually draws indicator
    void DrawIndicator () {
        // effectively erase the button before drawing the un-pressed version
        LCD.SetFontColor (backgroundColor);
        LCD.FillRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y), box.width, box.height );
        // draw the actual button
        LCD.SetFontColor (textColor);
        LCD.SetBackgroundColor (backgroundColor);
        LCD.WriteAt ( text, (int)(box.points[0].x+BUTT_MARGIN), -(int)(box.points[0].y - box.height / 2.0 + TEXT_HEIGHT / 2.0) );
        LCD.SetFontColor (borderColor);
        LCD.DrawRectangle ( (int)(box.points[0].x), -(int)(box.points[0].y), box.width, box.height );
    }
};


class CourseObjects {
public:
    CourseObjects () {
        bounds = Box (Vector2 (0, 0), COURSE_WIDTH, COURSE_HEIGHT);
        
        bumb = Box (Vector2 (1, -340), 232, 11);
        discoBall = Circle (Vector2(155.5, -161), 25);
        obstacleBox = Box (Vector2 (174, -351+999999), 59, 39); // 175
        
        DDRPad = Box (Vector2 (214, -569), 97, 48);
        DDRButtons = Box (Vector2 (214, -636), 97, 48);
        rightRamp = Box (Vector2 (232, -258), 116, 240);
        
        foosball = Box (Vector2 (155, -2), 166, 39);
        leverBank = Edge (Vector2 (1, -124), Vector2 (123, -2));
        leverBank.FlipPoints ();
        tokenSlot = Box (Vector2 (116, -351), 59 + 59, 39); // tokenSlot = Box (Vector2 (116, -351), 59, 39);
        tokenBowl = Circle (Vector2(146, -403.5), 22);
        
        leftRamp = Box (Vector2 (1, -351), 115, 87);
        finalBank = Edge (Vector2 (1, -617), Vector2 (83, -699));
        
        boxCount = 0;
        // boxes [boxCount++] = ToRealObject (bumb);
        boxes [boxCount++] = ToRealObject (obstacleBox);
        boxes [boxCount++] = ToRealObject (DDRButtons);
        // boxes [boxCount++] = ToRealObject (rightRamp);
        boxes [boxCount++] = ToRealObject (foosball);
        boxes [boxCount++] = ToRealObject (tokenSlot);
        // boxes [boxCount++] = ToRealObject (leftRamp);
        edgeCount = 0;
        edges [edgeCount++] = ToRealObject (leverBank);
        edges [edgeCount++] = ToRealObject (finalBank);
        invertedBoxCount = 0;
        invertedBoxes [invertedBoxCount++] = ToRealObject (bounds);
    }
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
    
    static const int SHAPE_COUNT = 64;
    int edgeCount;
    int boxCount;
    int invertedBoxCount;
    Edge edges [SHAPE_COUNT];
    Box boxes [SHAPE_COUNT];
    Box invertedBoxes [1];
    
    constexpr static float COURSE_PIXELS_TO_INCHES = 1.0 / 9.72222222222;
    static const int COURSE_WIDTH = 350;
    static const int COURSE_HEIGHT = 700;
    
    // conpoints a local position relative to the course to a global position
    Vector2 ToRealPosition (Vector2 local) {
        // Vector2 global = Vector2 (local.x * COURSE_PIXELS_TO_INCHES, local.y * COURSE_PIXELS_TO_INCHES);
        Vector2 global = Vector2 (local.x * COURSE_PIXELS_TO_INCHES, local.y * COURSE_PIXELS_TO_INCHES);
        return global;
    }
    // convert course object in pixel coordinates real world coordinates (inches)
    Box ToRealObject (Box obj) {
        // convert the box to global coordinates
        Box globalObj = Box ( ToRealPosition (obj.points[0]), obj.width * COURSE_PIXELS_TO_INCHES, obj.height * COURSE_PIXELS_TO_INCHES);
        // return the global box
        return globalObj;
    }
    Edge ToRealObject (Edge obj) {
        // convert the box to global coordinates
        Edge globalObj = Edge ( ToRealPosition (obj.points[0]), ToRealPosition (obj.points[1]), obj.dir);
        // return the global box
        return globalObj;
    }
private:
};




