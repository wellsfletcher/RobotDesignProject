#define IS_SIMULATION 1
/*



 SHARED simulation & non-simulation stuff



*/






// include statements

#include "Classes.hpp"
using namespace std;

// important global variable declarations

extern Vehicle simulatedVehicle; // this declaration enables the use of the simulatedVehicle object across files






#if !IS_SIMULATION // if this is not the simulation, then this chunck of code is used. Otherwise, this code is ignored.
/*



 NON-simulation only stuff



*/






class Simulation {
public:
    Simulation (Vehicle *vehicle) {
        veh = vehicle;
    }
    Simulation () {

    }
    Vehicle *veh;

    void Update () {

    }
private:
};






#endif
#if IS_SIMULATION // if this is the simulation, then this chunck of code is used. Otherwise, this code is ignored.
/*



 SIMULATION only stuff



 */






// include statements

#ifndef Simulation_hpp
#define Simulation_hpp
#include <stdio.h>

// #include "GFWMotor.hpp"


/************************************************************************ GLOBAL SIMULATION VARIABLE DECLERATIONS ************************************************************************/


/************************ IMPORTANT VARIABLES ************************/

// ...


/************************ PHYSICS CLASSES ************************/

// class that contains a bunch off arrays of pointers to different shape objects; used to specify which objects a physics object can collide with
class LayerMask {
public:
    LayerMask (LayerMask *tempLM) {
        edgeCount = 0;
        boxCount = 0;
        invertedBoxCount = 0;
        polygonCount = 0;
    }
    LayerMask () {
        edgeCount = 0;
        boxCount = 0;
        invertedBoxCount = 0;
        polygonCount = 0;
    }
    static const int MAX_EDGES = 64;
    static const int MAX_BOXES = 64;
    static const int MAX_INVERTED_BOXES = 1;
    static const int MAX_POLYGONS = 64;
    int edgeCount;
    int boxCount;
    int invertedBoxCount;
    int polygonCount;
    Edge edges [MAX_EDGES];
    Box boxes [MAX_BOXES];
    Box invertedBoxes [MAX_INVERTED_BOXES];
    Polygon polygons [MAX_POLYGONS];
    
    // sets the edge shape objects to be included in the layer mask
    void SetEdges (Edge tempEdges [MAX_EDGES], int tempEdgeCount) {
        edgeCount = tempEdgeCount;
        for (int k = 0; k < edgeCount; k++) {
            edges [k] = tempEdges [k];
        }
    }
    // sets the box shape objects to be included in the layer mask
    void SetBoxes (Box tempBoxes [MAX_BOXES], int tempBoxCount) {
        boxCount = tempBoxCount;
        for (int k = 0; k < boxCount; k++) {
            boxes [k] = tempBoxes [k];
        }
    }
    // sets the inverted box shape objects to be included in the layer mask
    void SetInvertedBoxes (Box tempBoxes [MAX_INVERTED_BOXES], int tempBoxCount) {
        invertedBoxCount = tempBoxCount;
        for (int k = 0; k < invertedBoxCount; k++) {
            invertedBoxes [k] = tempBoxes [k];
        }
    }
    // sets the edge shape objects to be included in the layer mask
    void SetPolygons (Polygon tempObjs [MAX_POLYGONS], int tempCount) {
        polygonCount = tempCount;
        for (int k = 0; k < tempCount; k++) {
            polygons [k] = tempObjs [k];
        }
    }
    // converts the given layer mask to a layer mask containing only polygons
    const static LayerMask ConvertToPolygonMask (LayerMask LM) {
        LayerMask polyMask = LayerMask ();
        int count;
        int endCount;
        
        count = 0;
        endCount = count + LM.polygonCount;
        
        for (int k = count; k < endCount; k++) {
            polyMask.polygons [k] = Polygon (LM.polygons [k - count]);
        }
        
        count = endCount; // - 1
        endCount = count + LM.invertedBoxCount;
        
        for (int k = count; k < endCount; k++) {
            // polyMask.polygons [k] = Polygon::ConvertToPolygon (LM.invertedBoxes [k]);
            polyMask.polygons [k] = Polygon::ConvertToFlippedPolygon (LM.invertedBoxes [k - count]);
        }
        
        count = endCount;
        endCount = count + LM.boxCount;
        
        for (int k = count; k < endCount; k++) {
            polyMask.polygons [k] = Polygon::ConvertToPolygon (LM.boxes [k - count]);
        }
        
        count = endCount;
        endCount = count + LM.edgeCount;
        
        for (int k = count; k < endCount; k++) {
            polyMask.polygons [k] = Polygon::ConvertToPolygon (LM.edges [k - count]);
        }
        
        polyMask.polygonCount = endCount;
        
        return polyMask;
    }
private:
};


class Raycast {
public:
    // Raycast (Vector2 origin, Vector2 direction, float maxDistance = 99999, char type = 'X');
    Raycast (LayerMask tempLM, Vector2 origin, Vector2 direction, float maxDistance, char type) {
        LM = tempLM; // note that it doesn't currently do a deep copy
        stepSize = 0.1; // was 1.0
        hit = false;
        colliderType = 'X';
        point = Vector2 ();
        distance = 0;
        float currentDistance = 0; // current distance of the particle
        
        pos = Vector2 (origin.x, origin.y); // deep copy the origin
        // convert direction into a unit vector * stepSize
        dir = direction.getUnitVector ();
        dir = Vector2 (dir.x * stepSize, dir.y * stepSize);
        
        // cout << "Particle bnds x = " << playerObj.bounds.points [0].x << ", y = " << playerObj.bounds.points [0].y;
        //- playerObj.bounds.UpdatePosition (playerObj.pos); // update the bounds of the player    // may already be done else where
        // cout << "; new bnds x = " << playerObj.bounds.points [0].x << ", y = " << playerObj.bounds.points [0].y << endl;
        
        // iterate through all the collision object types and check for collision with them
        while (hit == false && currentDistance < maxDistance) {
            currentDistance = Distance (origin, pos);
            if (!hit) {
                // cout << "oi" << endl;
                // check for collision with boxes
                for (int j = 0; j < LM.invertedBoxCount && !hit; j++) {
                    if (CheckInvertedBoxCollision (LM.invertedBoxes [j])) { // if there was a collision then
                        // cout << "oi " << TimeNow () << endl;
                        hit = true; // note that there was a collision
                        point = pos;
                        distance = Distance (origin, point); // or currentDistance
                        colliderType = 'i'; // collider type equals edge
                        break;
                    }
                }
                
            } if (!hit) {
                
                // check for collision with boxes
                for (int j = 0; j < LM.boxCount && !hit; j++) {
                    if (CheckBoxCollision (LM.boxes [j])) { // if there was a collision then
                        hit = true; // note that there was a collision
                        point = pos;
                        distance = Distance (origin, point); // or currentDistance
                        colliderType = 'b'; // collider type equals edge
                        break;
                    }
                }
                
            } if (!hit) {
                
                // check for collision with edges
                for (int j = 0; j < LM.edgeCount && !hit; j++) { // CollideEdge (tunnelEdges [j]);
                    LM.edges [j].normal = LM.edges [j].getNormal (); // fite me
                    if (CheckEdgeCollision (LM.edges [j])) { // if there was a collision then
                        // cout << j;
                        hit = true; // note that there was a collision
                        point = pos;
                        distance = Distance (origin, point); // or currentDistance
                        colliderType = 'e'; // collider type equals edge
                        break; // may not actually be breaking fully
                    }
                }
                
            }
            // increment the particle
            pos = Vector2 (pos.x + dir.x, pos.y + dir.y);
            // point = pos;
            // cout << "Particle x = " << pos.x << ", y = " << pos.y << endl;
            // LCD.SetFontColor (RED);
            // LCD.FillCircle (pos.x, -pos.y, 5);
        }
        if (hit == false && currentDistance >= maxDistance) { // if the loop exited due to the distance limit being exceeded
            point = pos;
        }
    }
    Raycast () {
        
    }
    bool hit; // whether the ray hit or not
    char colliderType; // type of collider hit the ray
    // Shape collider; // the collider hit by the ray
    Vector2 normal; // the normal of the surface hit by the ray
    Vector2 point; // the point in which the ray hit the collider's surface
    float distance; // the distance from the origin to where the ray hit the collider's surface
private:
    float stepSize; // the step size per iteration of the raycast
    Vector2 pos; // current position of the particle
    Vector2 dir; // standarized direction of the raycast
    LayerMask LM; // a layer mask denoting the shape objects in which the raycast can collide with
    // checks if the bullet has collided with a box collision object
    bool CheckBoxCollision (Box obj) {
        bool collided = false;
        // convert the box into collision edges
        Edge southEdge = Edge (obj.points[2], obj.points[3], 0);
        Edge westEdge = Edge (obj.points[0], obj.points[2], 1);
        Edge northEdge = Edge (obj.points[0], obj.points[1], 2);
        Edge eastEdge = Edge (obj.points[1], obj.points[3], 3);
        
        Edge boxEdges [4];
        int boxEdgeCount = 0;
        boxEdges [boxEdgeCount++] = southEdge;
        boxEdges [boxEdgeCount++] = westEdge;
        boxEdges [boxEdgeCount++] = northEdge;
        boxEdges [boxEdgeCount++] = eastEdge;
        
        for (int k = 0; !collided && k < boxEdgeCount; k++) {
            if (CheckEdgeCollision (boxEdges [k])) {
                collided = true;
                // normal = Vector2 (boxEdges [k].norm.x, boxEdges [k].norm.y);
            }
        }
        
        return collided;
    }
    bool CheckInvertedBoxCollision (Box obj) {
        bool collided = false;
        
        // convert the box into collision edges
        Edge southEdge = Edge (obj.points[2], obj.points[3], 2);
        Edge westEdge = Edge (obj.points[0], obj.points[2], 3);
        Edge northEdge = Edge (obj.points[0], obj.points[1], 0);
        Edge eastEdge = Edge (obj.points[1], obj.points[3], 1);
        
        Edge boxEdges [4];
        int boxEdgeCount = 0;
        boxEdges [boxEdgeCount++] = southEdge;
        boxEdges [boxEdgeCount++] = westEdge;
        boxEdges [boxEdgeCount++] = northEdge;
        boxEdges [boxEdgeCount++] = eastEdge;
        
        for (int k = 0; !collided && k < boxEdgeCount; k++) {
            if (CheckEdgeCollision (boxEdges [k])) {
                collided = true;
                // normal = Vector2 (boxEdges [k].norm.x, boxEdges [k].norm.y);
            }
        }
        
        /*
         bool sCol = CheckEdgeCollision (southEdge);
         bool wCol = CheckEdgeCollision (westEdge);
         bool nCol = CheckEdgeCollision (northEdge);
         bool eCol = CheckEdgeCollision (eastEdge);
         
         if (nCol) {
         int n = 0;
         }
         if (sCol) {
         int n = 0;
         }
         */
        
        return collided;
        
        /*
         if (CheckEdgeCollision (southEdge) || CheckEdgeCollision (westEdge) || CheckEdgeCollision (northEdge) || CheckEdgeCollision (eastEdge)) {
         return true;
         } else {
         return false;
         }
         */
    }
    bool CheckEdgeCollision (Edge edge) {
        Vector2 point = pos;
        float EDGE_TOL = 2.0; // initialize edge tolerance, which is essentially a pseudo width for collision edges
        bool collided = false;
        // edge.normal = edge.getNormal ();
        
        // float scalTestBoi = Vector2::Scal (Vector2 (1, 0), Vector2 (2, 2));
        // float scalTestBoi = Vector2::Scal (Vector2 (1, 2), Vector2 (0, 0));
        
        // float velocityScal = Vector2::Scal (edge.normal, physObj.vel);
        // if the physics object is going against the direction of the edge's normal vector
        // (if proj (edge's normal vector, physObj's velocity) is the opposite sign of the normal vector ... if scal (edge's normal vector, physObj's velocity) is negative)
        // if (velocityScal < 0) {
        Vector2 pointRelativeToEdge0 = Vector2 (point.x - edge.points [0].x, point.y - edge.points [0].y);
        Vector2 dirEdge0ToEdge1 = Vector2::SlopeVect (edge.points [0], edge.points [1]);
        float xScal0 = Vector2::Scal (pointRelativeToEdge0, dirEdge0ToEdge1);
        
        Vector2 pointRelativeToEdge1 = Vector2 (point.x - edge.points [1].x, point.y - edge.points [1].y);
        Vector2 dirEdge1ToEdge0 = Vector2::SlopeVect (edge.points [1], edge.points [0]);
        float xScal1 = Vector2::Scal (pointRelativeToEdge1, dirEdge1ToEdge0);
        // if the point is in between the two "X" points denoting the edge (if you pretend the edge is the x-axis and its normal vector is the y-axis)
        // (if scal (the point's position relative to the edge's first point, the direction from the edge's first point to its second point) is positive)
        // (and if scal (the point's position relative to the edge's second point, the direction from the edge's second point to its first point) is positive)
        if (xScal0 > 0 && xScal1 > 0) { // maybe should be >= ... most likely doesn't matter that much
            
            // the imaginary point is calculated by subtracting the edge's unit normal vector * EDGE_TOL from the edge's first point
            Vector2 imaginaryPoint = Vector2 (edge.points [0].x - edge.normal.x * EDGE_TOL, edge.points [0].y - edge.normal.y * EDGE_TOL);
            
            pointRelativeToEdge0 = pointRelativeToEdge0;
            Vector2 dirEdge0ToImaginary = Vector2::SlopeVect (edge.points [0], imaginaryPoint);
            float yScal0 = Vector2::Scal (pointRelativeToEdge0, dirEdge0ToImaginary);
            
            Vector2 pointRelativeToImaginary = Vector2 (point.x - imaginaryPoint.x, point.y - imaginaryPoint.y);
            Vector2 dirImaginaryToEdge0 = Vector2::SlopeVect (imaginaryPoint, edge.points [0]);
            float yScal1 = Vector2::Scal (pointRelativeToImaginary, dirImaginaryToEdge0);
            
            // if the point is in between the two (imaginary) "Y" points denoting the edge (pretend the edge has an imaginary depth (so the edge basically a box); the two points would be the first point of the actual edge, and that point with the imaginary depth added onto it (where that imaginary depth equals EDGE_TOL))
            // (if scal (the point's position relative to the edge's first point, the direction from the edge's first point to the imaginary point) is positive)
            // (and if scal (the point's position relative to the imaginary point, the direction from the imaginary point to the edge's first point) is positive)
            if (yScal0 > 0 && yScal1 > 0) {
                // then the point has indeed collided with the edge!
                collided = true; // indicate that it be that way
            }
            
            // }
        }
        
        return collided; // collided true
    }
    // calculates distance between two points
    float Distance (Vector2 start, Vector2 end) {
        return sqrt (  pow (end.x - start.x, 2.0)  +  pow (end.y - start.y, 2.0)  );
    }
};


/************************************************************************ SIMULATION CODE ************************************************************************/

class PhysicsObject {
public:
    PhysicsObject (Polygon tempShape, Vector2 tempPos, Vector2 tempVel, float tempAngVel, float tempMass) {
        shape = Polygon (tempShape);
        pos = Vector2 (tempPos.x, tempPos.y);
        vel = Vector2 (tempVel.x, tempVel.y);
        angVel = tempAngVel;
        
        mass = tempMass;
    }
    PhysicsObject () {
        
    }
    /*
    PhysicsObject (float tempMass) {
        mass = tempMass;
    }
    PhysicsObject () {
        
    }
    void UpdateVars (Polygon tempShape, Vector2 tempPos, Vector2 tempVel) {
        shape = Polygon (tempShape);
        pos = Vector2 (tempPos.x, tempPos.y);
        vel = Vector2 (tempVel.x, tempVel.y);
    }
    */
    Polygon shape;
    Vector2 pos;
    Vector2 vel;
    float angVel;
    float mass;
private:
};


// // physics system with one dynamic physics object and several static objects
// in retrospect, I should have set this up more like how the raycast object is
class PhysicsCalculator {
public:
    PhysicsCalculator (LayerMask tempLM, PhysicsObject tempPhysObj) {
        LM = tempLM; // LM should only contain polygons
        physObj = tempPhysObj;
        // statCount = 0;
        
        force = Vector2 (0, 0);
        torque = 0;
    }
    PhysicsCalculator () {

    }
    LayerMask LM;
    PhysicsObject physObj;
    /*
    const static int MAX_STATIC_OBJECTS = 64;
    int statCount;
    Polygon statObjs [MAX_STATIC_OBJECTS];
    */
    // temporary variables set after collision
    Vector2 force;
    float torque;
    
    /*
    void SetStaticObjects (Polygon tempStatObjs [MAX_STATIC_OBJECTS], int tempStatCount) {
        statCount = tempStatCount;
        for (int k = 0; k < statCount; k++) {
            statObjs [k] = tempStatObjs [k];
        }
    }
    */
    void UpdatePhysicsObjectVars (Polygon tempShape, Vector2 tempPos, Vector2 tempVel, float tempAngVel) {
        Polygon newPoly = Polygon (tempShape);
        newPoly.Translate (tempPos);
        physObj = PhysicsObject (newPoly, tempPos, tempVel, tempAngVel, physObj.mass);
    }
    void CalculateCollisions () {
        bool collided = false;
        // reset vars
        torque = 0;
        force = Vector2 (0, 0);
        
        // check for collision with edges
        for (int k = 0; k < LM.polygonCount; k++) { // CollideEdge (tunnelEdges [j]);
            collided = collided || CollideWithPolygon (LM.polygons [k]);
        }
        /*
        // reset vars if there wasn't a collision
        if (!collided) {
            torque = 0;
            force = Vector2 (0, 0);
        }
        */
    }
private:
    // create constants
    constexpr static float EDGE_TOL = 2.0; // edge tolerance
    // simulates collision between a this.physObj and the given static polygon
    // updates this.force and this.torque to be the force and torque applied to the physics due to the collision
    // returns whether there was a collision
    bool CollideWithPolygon (Polygon poly) {
        bool collided = false;
        
        // calculate collision between physics object points and static object (polygon) edges
        
        Polygon pointPoly = physObj.shape;
        Polygon edgePoly = poly;
        
        for (int i = 0; i < pointPoly.length; i++) {
            Vector2 point = pointPoly.points [i];
            for (int k = 0; k < edgePoly.length; k++) { // edgePoly.length-1
                Edge edge;
                if (k != edgePoly.length - 1) {
                    edge = Edge (   Vector2 (edgePoly.points [k].x, edgePoly.points [k].y),  Vector2 (edgePoly.points [k+1].x, edgePoly.points [k+1].y)   );
                } else if (k > 2) { // if it is a real polygon...
                    edge = Edge (   Vector2 (edgePoly.points [k].x, edgePoly.points [k].y),  Vector2 (edgePoly.points [0].x, edgePoly.points [0].y)   );
                }
                bool justCollided = CheckCollision (point, edge);
                
                if (justCollided) {
                    collided = true;
                    // // reset torque and force variables (to make them not compound ... should change this in the future)
                    // torque = 0;
                    // force = Vector2 (0, 0);
                    
                    // do physics calculations
                    CalculateForces (point, edge, false);
                }
            }
        }
        
        // calculate collision between physics object edges and static object (polygon) points
        
        pointPoly = poly;
        edgePoly = physObj.shape;
        
        for (int i = 0; i < pointPoly.length; i++) {
            Vector2 point = pointPoly.points [i];
            for (int k = 0; k < edgePoly.length; k++) { // edgePoly.length-1
                Edge edge;
                if (k != edgePoly.length - 1) {
                    edge = Edge (   Vector2 (edgePoly.points [k].x, edgePoly.points [k].y),  Vector2 (edgePoly.points [k+1].x, edgePoly.points [k+1].y)   );
                } else if (k > 2) { // if it is a real polygon...
                    edge = Edge (   Vector2 (edgePoly.points [k].x, edgePoly.points [k].y),  Vector2 (edgePoly.points [0].x, edgePoly.points [0].y)   );
                }
                bool justCollided = CheckCollision (point, edge);
                
                if (justCollided) {
                    collided = true;
                    // // reset torque and force variables (to make them not compound ... should change this in the future)
                    // torque = 0;
                    // force = Vector2 (0, 0);
                    
                    // do physics calculations
                    CalculateForces (point, edge, true);
                }
            }
        }
        
        return collided;
    }
    
    // updates this.force and this.torque
    void CalculateForces (Vector2 point, Edge edge, bool isFlipped) {
        int flipper = 1;
        // if the calculated forces should be flipped ...
        if (isFlipped) {
            flipper = -1;
        }
        float MAGIC_ANG_ACCEL_NUMBER = 2.66; // 16.6; // 2.66; // 1.66
        float RESTI = 2.0; // (kind of) coefficient of restitution // 2.0
        // force = Vector2 (edge.normal.x * abs (physObj.vel.x) * RESTI, edge.normal.y * abs (physObj.vel.y) * RESTI);
        force = Vector2 (force.x + edge.normal.x * abs (physObj.vel.x) * (RESTI * flipper), force.y + edge.normal.y * abs (physObj.vel.y) * (RESTI * flipper));
        
        Vector2 angForce = Vector2 (edge.normal.x * (0.1 * RESTI * MAGIC_ANG_ACCEL_NUMBER* flipper), edge.normal.y * (0.1 * RESTI * MAGIC_ANG_ACCEL_NUMBER * flipper)); // hmmm
        Vector2 radialPosition = Vector2 (point.x - physObj.pos.x, point.y - physObj.pos.y);
        torque = torque + radialPosition.CalculateCrossProduct (angForce);
        
        // cout << "Raycast normal = " << edge.normal.x << ", " << edge.normal.y << endl;
    }
    
    // checks if the given point has collided with the given edge
    bool CheckCollision (Vector2 point, Edge edge) {
        bool collided = false;
        
        // float scalTestBoi = Vector2::Scal (Vector2 (1, 0), Vector2 (2, 2));
        // float scalTestBoi = Vector2::Scal (Vector2 (1, 2), Vector2 (0, 0));
        
        float velocityScal = Vector2::Scal (edge.normal, physObj.vel);
        // if the physics object is going against the direction of the edge's normal vector
        // (if proj (edge's normal vector, physObj's velocity) is the opposite sign of the normal vector ... if scal (edge's normal vector, physObj's velocity) is negative)
        // if (velocityScal < 0) {
            Vector2 pointRelativeToEdge0 = Vector2 (point.x - edge.points [0].x, point.y - edge.points [0].y);
            Vector2 dirEdge0ToEdge1 = Vector2::SlopeVect (edge.points [0], edge.points [1]);
            float xScal0 = Vector2::Scal (pointRelativeToEdge0, dirEdge0ToEdge1);
            
            Vector2 pointRelativeToEdge1 = Vector2 (point.x - edge.points [1].x, point.y - edge.points [1].y);
            Vector2 dirEdge1ToEdge0 = Vector2::SlopeVect (edge.points [1], edge.points [0]);
            float xScal1 = Vector2::Scal (pointRelativeToEdge1, dirEdge1ToEdge0);
            // if the point is in between the two "X" points denoting the edge (if you pretend the edge is the x-axis and its normal vector is the y-axis)
            // (if scal (the point's position relative to the edge's first point, the direction from the edge's first point to its second point) is positive)
            // (and if scal (the point's position relative to the edge's second point, the direction from the edge's second point to its first point) is positive)
            if (xScal0 > 0 && xScal1 > 0) { // maybe should be >= ... most likely doesn't matter that much
            
                // the imaginary point is calculated by subtracting the edge's unit normal vector * EDGE_TOL from the edge's first point
                Vector2 imaginaryPoint = Vector2 (edge.points [0].x - edge.normal.x * EDGE_TOL, edge.points [0].y - edge.normal.y * EDGE_TOL);
                
                pointRelativeToEdge0 = pointRelativeToEdge0;
                Vector2 dirEdge0ToImaginary = Vector2::SlopeVect (edge.points [0], imaginaryPoint);
                float yScal0 = Vector2::Scal (pointRelativeToEdge0, dirEdge0ToImaginary);
                
                Vector2 pointRelativeToImaginary = Vector2 (point.x - imaginaryPoint.x, point.y - imaginaryPoint.y);
                Vector2 dirImaginaryToEdge0 = Vector2::SlopeVect (imaginaryPoint, edge.points [0]);
                float yScal1 = Vector2::Scal (pointRelativeToImaginary, dirImaginaryToEdge0);
                
                // if the point is in between the two (imaginary) "Y" points denoting the edge (pretend the edge has an imaginary depth (so the edge basically a box); the two points would be the first point of the actual edge, and that point with the imaginary depth added onto it (where that imaginary depth equals EDGE_TOL))
                // (if scal (the point's position relative to the edge's first point, the direction from the edge's first point to the imaginary point) is positive)
                // (and if scal (the point's position relative to the imaginary point, the direction from the imaginary point to the edge's first point) is positive)
                if (yScal0 > 0 && yScal1 > 0) {
                    // then the point has indeed collided with the edge!
                    collided = true; // indicate that it be that way
                }
            
            // }
        }
        
        return collided; // collided true
    }
    
    // check for collision between the points of the first polygon and the edges of the second polygon
    bool CheckCollisionBetweenPolygonEdgesAndPoints (Polygon pointPoly, Polygon edgePoly) {
        bool collided = false;
        
        for (int i = 0; !collided && i < pointPoly.length; i++) {
            Vector2 point = pointPoly.points [i];
            for (int k = 0; !collided && k < edgePoly.length-1; k++) {
                Edge edge = Edge (   Vector2 (edgePoly.points [k].x, edgePoly.points [k].y),  Vector2 (edgePoly.points [k+1].x, edgePoly.points [k+1].y)   );
                collided = CheckCollision (point, edge);
            }
        }
        
        return collided; // ope this method would also need to return the collision point and edge to be useful
    }
};


class Simulation {
public:
    Simulation (Vehicle *vehicle) {
        veh = vehicle;
        
        // setup layermasks for raycasts and physics stuff
        SetUpPhysics ();
        
        timeSinceUpdate = TimeNow ();
    }
    Simulation () {

    }
    Vehicle *veh;
    
    // set up all of the physics stuff
    void SetUpPhysics () {
        // set up layer masks (for bump switches)
        
        courseObjects = CourseObjects ();
        
        int edgeCount = courseObjects.edgeCount;
        int boxCount = courseObjects.boxCount;
        int invertedBoxCount = courseObjects.invertedBoxCount;
        
        Box invertedBoxes [LayerMask::MAX_INVERTED_BOXES];
        Box boxes [LayerMask::MAX_BOXES];
        Edge edges [LayerMask::MAX_EDGES];
        
        for (int k = 0; k < edgeCount; k++) {
            edges [k] = courseObjects.edges [k];
        }
        for (int k = 0; k < boxCount; k++) {
            boxes [k] = courseObjects.boxes [k];
        }
        for (int k = 0; k < invertedBoxCount; k++) {
            invertedBoxes [k] = courseObjects.invertedBoxes [k];
        }
        
        // edgeCount = 0; // temporary
        
        bumpLM = LayerMask ();
        bumpLM.SetEdges (edges, edgeCount);
        bumpLM.SetBoxes (boxes, boxCount);
        bumpLM.SetInvertedBoxes (invertedBoxes, invertedBoxCount);
        
        // setup physics system
        physicsLM = LayerMask::ConvertToPolygonMask (bumpLM);
        float physObjMass = 1.0;
        PhysicsObject physObj = PhysicsObject (veh->chassis, veh->pos, veh->vel, veh->angVel, physObjMass);
        physics = PhysicsCalculator (physicsLM, physObj);
    }
    void Update () {
        /*
        // get bump switch value
        for (int k = 0; k < veh->bumpsLength; k++) {
            veh->bumps [k].SetValue (SimulateBumpValue (&veh->bumps [k]));
        }
        */
        float deltaTime = TimeNow () - timeSinceUpdate; // the elapsed amount of time in sseconds
        timeSinceUpdate = TimeNow ();

        Vector2 netVelocity;
        float netAngularVelocity = 0;
        
        // add up velocities of all the wheels (powerPercents * wheel direction * max velocity)
        // add up the angular velocities caused by all the wheels (calculated similar to torque)
        for (int k = 0; k < veh->wheelsLength; k++) {
            float percentMotorPower = veh->wheels [k].activePercent;
            float wheelVelocityMagnitude = percentMotorPower * MAX_VELOCITY * deltaTime;
            // float wheelAngularVelocityMagnitude = percentMotorPower * MAX_ANGULAR_VELOCITY * deltaTime;
            float MAGIC_VEL_NUMBER = 1 / 1.66;
            
            Vector2 wheelDirection = veh->wheels [k].dir.getUnitVector();
            Vector2 wheelVelocity = Vector2 (wheelDirection.x * wheelVelocityMagnitude * MAGIC_VEL_NUMBER, wheelDirection.y * wheelVelocityMagnitude * MAGIC_VEL_NUMBER);
            netVelocity = Vector2 (netVelocity.x + wheelVelocity.x, netVelocity.y + wheelVelocity.y);
            
            float MAGIC_ANG_ACCEL_NUMBER = 1.66;
            // float MAGIC_ANG_ACCEL_NUMBER = 1.66 * deltaTime * 280;
            float angularVelocity = -veh->radius * wheelVelocityMagnitude * MAGIC_ANG_ACCEL_NUMBER; // hmmm
            netAngularVelocity += angularVelocity;
            veh->wheels [k].lastForceApplied = wheelVelocity;
        }
        
        veh->lastForceApplied = netVelocity;
        // use accelerateTo function to accelerate to the target velocity
        // do similarly with angular velocity
        float gain = 1.0;
        AccelerateTo (netVelocity, gain);
        AngularAccelerateTo (netAngularVelocity, gain);

        /*
        veh->UpdatePosition ();
        veh->UpdateRotation ();
        */
    
        // for (int k = 0; k < 999999; k++); // purposely slow down the simulation to see how it is affected; it does affect it, which is neat and unfortunate
        // timeSinceUpdate = TimeNow ();
        
        physics.UpdatePhysicsObjectVars (veh->chassis, veh->pos, veh->vel, veh->angVel);
        physics.CalculateCollisions ();
        Vector2 collisionForce = physics.force;
        float collisionTorque = physics.torque;
        veh->ApplyAcceleration (collisionForce);
        veh->ApplyAngularAcceleration (collisionTorque);
        
        // get bump switch value
        for (int k = 0; k < veh->bumpsLength; k++) {
            veh->bumps [k].SetValue (SimulateBumpValue (&veh->bumps [k]));
        }
        
        veh->UpdatePosition ();
        veh->UpdateRotation ();
    }
    bool SimulateBumpValue (BumpSwitch *bump) {
        // perform ray casts
        Vector2 origin = Vector2 (veh->pos.x + bump->pos.x, veh->pos.y + bump->pos.y); // 72 is course height
        bump->startPos = origin;
        Vector2 direction = bump->dir;
        float maxDistance = 0.0001; // 1.0
        int uselessTypeID = 'a';
        Raycast raycast = Raycast (bumpLM, origin, direction, maxDistance, uselessTypeID);
        // set bump endpoint position to where the raycast ended
        bump->endPos = raycast.point;
        
        // apply velocity to vehicle using normal vector if hit
        if (raycast.hit) {
            /*
            float MAGIC_ANG_ACCEL_NUMBER = 2.66; // 1.66
            float RESTI = 2.0; // (kind of) coefficient of restitution // 2.0
            Vector2 force = Vector2 (raycast.normal.x * abs (veh->vel.x) * RESTI, raycast.normal.y * abs (veh->vel.y) * RESTI);
            veh->ApplyAcceleration (force);
            
            // Vector2 angForce = Vector2 (raycast.normal.x * abs (veh->vel.x) * RESTI, raycast.normal.y * abs (veh->vel.y) * RESTI);
            Vector2 angForce = Vector2 (raycast.normal.x * 0.1 * RESTI, raycast.normal.y * 0.1 * RESTI); // hmmm
            // Vector2 angForce = Vector2 (raycast.normal.x * veh->vel.x * RESTI, raycast.normal.y * veh->vel.y * RESTI);
            // Vector2 angForce = Vector2 (veh->vel.x * RESTI, veh->vel.y * RESTI);
            // angForce = Vector2 (angForce.y, -angForce.x);
            Vector2 radialPosition = Vector2 (raycast.point.x - veh->pos.x, raycast.point.y - veh->pos.y);
            float torque = radialPosition.CalculateCrossProduct (angForce);
            veh->ApplyAngularAcceleration (torque * MAGIC_ANG_ACCEL_NUMBER);
            
            cout << "Raycast normal = " << raycast.normal.x << ", " << raycast.normal.y << endl;
            */
        }
        // return raycast value
        
        return !(raycast.hit);
        // return ((int)(bump.pos.x)) % 2;
    }
    void AccelerateTo (Vector2 targetVelocity, float gain) {
        Vector2 accel = Vector2 (targetVelocity.x - veh->vel.x, targetVelocity.y - veh->vel.y);
        veh->ApplyAcceleration (accel);
    }
    void AngularAccelerateTo (float targetVelocity, float gain) {
        float angAccel = targetVelocity - veh->angVel;
        veh->ApplyAngularAcceleration (angAccel);
    }
    float ApplyFrictionalForce (float force, float friction) {
        if (force > 0) {
            force = force - friction;
            if (force < 0) {
                force = 0;
            }
        } else if (force < 0) {
            force = force + friction;
            if (force > 0) {
                force = 0;
            }
        }
        return force;
    }
    float SignOf (float num) {
        int result = 1;
        if (num < 0) {
            result = -1;
        }
        return result;
    }
    float timeSinceUpdate; // the time passed since the simulated vehicle was updated
    
    // physics variables
    
    CourseObjects courseObjects;
    LayerMask bumpLM; // the layer mask for the bump switches
    LayerMask physicsLM; // the layer mask for physics collisions
    PhysicsCalculator physics;

    // constant variables initialization

    constexpr static float A_GRAVITY = 9.81;
    constexpr static float SQRT_3 = 1.73205080757;
    constexpr static float RADS_TO_DEGREES = M_PI / 180.0;
    constexpr static float METERS_TO_INCHES = 39.3701;
    constexpr static float INCHES_TO_METERS = 1.0 / METERS_TO_INCHES;
    
    constexpr static float VELOCITY_AT_50_POWER = 8.9; // in inches per second
    constexpr static float MAX_VELOCITY = VELOCITY_AT_50_POWER * 2.0; // in inches per second
    constexpr static float ANGULAR_VELOCITY_AT_50_POWER = 135.0; // in degrees per second
    constexpr static float MAX_ANGULAR_VELOCITY = ANGULAR_VELOCITY_AT_50_POWER * 2.0; // in degrees per second
private:
};




#endif /* Simulation_hpp */
#endif
