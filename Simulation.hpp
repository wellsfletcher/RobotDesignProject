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
#endif /* Simulation_hpp */

// #include "GFWMotor.hpp"


/************************************************************************ GLOBAL SIMULATION VARIABLE DECLERATIONS ************************************************************************/


/************************ IMPORTANT VARIABLES ************************/

// ...


/************************************************************************ SIMULATION CODE ************************************************************************/

class Simulation {
public:
    Simulation (Vehicle *vehicle) {
        veh = vehicle;
        timeSinceUpdate = TimeNow ();
    }
    Simulation () {

    }
    Vehicle *veh;

    void Update () {
        float deltaTime = TimeNow () - timeSinceUpdate; // the elapsed amount of time in sseconds

        Vector2 netForce = Vector2 (0, 0);
        float netTorque = 0;
        Vector2 netWheelPercentPower = Vector2 (0, 0);

        for (int k = 0; k < veh->wheelsLength; k++) {
            float motorPowerPercent = veh->wheels [k].activePercent;
            float wheelRadius = veh->wheels [k].radius;
            Vector2 wheelDirection = veh->wheels [k].dir;
            netWheelPercentPower = Vector2 (netWheelPercentPower.x + wheelDirection.x * motorPowerPercent, netWheelPercentPower.y + wheelDirection.y * motorPowerPercent); // for clamping magnitude later

            // I found how to do this bit at https://www.precisionmicrodrives.com/content/reading-the-motor-constants-from-typical-performance-characteristics/
            // it seems sketchy to me
            float SPEED_CONSTANT = NO_LOAD_SPEED / VOLTS; // angular speed per volt
            float wheelAngularVelocity = (VOLTS) * SPEED_CONSTANT; // if I could substitute this bit for an equation of the wheel's angular velocity over power percent that would be good methinks
            float power = VOLTS * CURRENT * motorPowerPercent;
            float motorTorque = power / wheelAngularVelocity;
            /*
            float motorTorque = ((NO_LOAD_SPEED * motorPowerPercent) * TORQUE_PER_RPS + STALL_TORQUE) - STALL_TORQUE; // sketch
            cout << "motorTorque " << motorTorque << endl;
            */

            float tractionForce = motorTorque * wheelRadius; // acording to omnidrive.pdf, tractionForce = wheelRadius * motor torque; seems sketchy to me
            float frictionForce = STAT_FRICT_FORCE;
            // frictionForce = 0; // comment out to enable friction
            float wheelForceMagnitude = ApplyFrictionalForce (tractionForce, frictionForce);
            Vector2 wheelForce = Vector2 (wheelDirection.x * wheelForceMagnitude, wheelDirection.y * wheelForceMagnitude);
            /*
            Vector2 tractionForce = Vector2 (wheelDirection.x * motorTorque * wheelRadius, wheelDirection.y * motorTorque * wheelRadius); // acording to omnidrive.pdf, tractionForce = wheelRadius * motor torque; seems sketchy to me
            Vector2 frictionForce = Vector2 (wheelDirection.x * STAT_FRICT_FORCE, wheelDirection.y * STAT_FRICT_FORCE);
            Vector2 wheelForce = Vector2 (ApplyFrictionalForce (tractionForce.x, frictionForce.x), ApplyFrictionalForce (tractionForce.y, frictionForce.y));
            */
            // Vector2 wheelForce = tractionForce;
            netForce = Vector2 (netForce.x + wheelForce.x, netForce.y + wheelForce.y);

            /*
            Vector2 rPos = veh->wheels [k].pos; // the position in which the torque is applied (r)
            float torque = rPos.x * wheelForce.y  -  rPos.y * wheelForce.x;
            netTorque = netTorque + torque;
            */
            float torque = veh->radius * (wheelForceMagnitude); // may not be exactly right because the wheels aren't really centered on (center of mass of) the vehicle
            netTorque = netTorque + torque;

            veh->wheels [k].lastForceApplied = wheelForce;
            // veh->wheels [k].lastForceApplied = Vector2 (-wheelForce.x, -wheelForce.y); // account for the direction of the motors being wrong
        }

        // (temporary) skip calculations and hardcode stuff; both of the below quantities are reasonable and the stuff calculated above should generally be similar to them
        // netForce = Vector2 (.08, .08);
        // netTorque = 10;

        Vector2 netAccel = Vector2 (netForce.x / MASS, netForce.y / MASS);
        float angularAccel = -netTorque / MoI;

        // convert back into correct units
        angularAccel = angularAccel * RADS_TO_DEGREES * deltaTime * deltaTime; // rad/s/s --> degrees/frame/frame
        netAccel = Vector2 (netAccel.x * METERS_TO_INCHES * deltaTime * deltaTime, netAccel.y * METERS_TO_INCHES * deltaTime * deltaTime); // m/s/s to inches/frame/frame

        // account for the direction of the motors being wrong
        /*
        angularAccel *= -1;
        netAccel = Vector2 (-netAccel.x, -netAccel.y);
        */

        veh->lastForceApplied = netForce;
        // veh->lastForceApplied = Vector2 (-netForce.x, -netForce.y); // account for the direction of the motors being wrong

        veh->ApplyAcceleration (netAccel);
        veh->ApplyAngularAcceleration (angularAccel);
        
        // clamp the magnitudes of the vehicles velocity and angular velocity (this currently makes acceleration irrelevent; will fix in the future)
        float percentWheelPowerUsed = netWheelPercentPower.getMagnitude (); // may have to seperate the amount rotation contributes to this... idk
        float maxVelocity = 0.02; // guess and check
        veh->vel.ClampMagnitude (percentWheelPowerUsed * maxVelocity);
        // cout << percentWheelPowerUsed << endl;
        // cout << "vel: " << veh->vel.x << ", " << veh->vel.y << endl;
        float maxAngularVelocity = 10;
        //veh->angVel = (veh->angVel / abs (veh->angVel)) * maxAngularVelocity;

        veh->UpdatePosition ();
        veh->UpdateRotation ();

        timeSinceUpdate = TimeNow ();
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
    float timeSinceUpdate; // the time passed since the simulated vehicle was updated

    // constant variables initialization

    constexpr static float A_GRAVITY = 9.81;
    constexpr static float SQRT_3 = 1.73205080757;
    constexpr static float RADS_TO_DEGREES = M_PI / 180.0;
    constexpr static float METERS_TO_INCHES = 39.3701;
    constexpr static float INCHES_TO_METERS = 1.0 / METERS_TO_INCHES;

    constexpr static float WHEEL_RADIUS = 1.2 * INCHES_TO_METERS; // veh->wheels[k].radius
    constexpr static float HEX_RADIUS = ((2.0*SQRT_3 + 1.5*SQRT_3) / 2.0) * INCHES_TO_METERS; // veh->radius

    constexpr static float RPM_TO_RPS = 2.0 * M_PI / 60.0; // RPM to radians per second
    constexpr static float OZIN_TO_NM = 0.0070615518333333; // ounce-inches to newton-meters

    constexpr static float NO_LOAD_SPEED = 112.0 * RPM_TO_RPS; // in radians per second
    constexpr static float STALL_TORQUE = 123.53 * OZIN_TO_NM; // in newton-meters
    constexpr static float TORQUE_PER_RPS = -1.0111 * OZIN_TO_NM / RPM_TO_RPS; // y = -1.011x + 123.53
    constexpr static float RPS_PER_TORQUE = 1.0 / TORQUE_PER_RPS;

    constexpr static float MASS = 1487.9 / 1000.0; // in kilograms; should probably get a more accurate measurement of this in the future
    constexpr static float WHEEL_MASS = (115.33) / 3000.0; // in kilograms; should probably get a more accurate measurement of this in the future
    constexpr static float MoI = 0.5 * MASS * HEX_RADIUS * HEX_RADIUS; // moment of intertia; should get this from SolidWorks in the future
    constexpr static float WHEEL_MoI = 0.5 * WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS; // wheel moment of intertia; should get this from SolidWorks in the future; this might be different due to wheel rotation w/ no slip

    constexpr static float STAT_FRICT_COEF = 0.1; // the static friction coefficient; this is currentrly arbatrary; I should probably get a better value
    constexpr static float NORMAL_FORCE = (MASS / 3.0) * A_GRAVITY; // the normal force on each wheel
    constexpr static float STAT_FRICT_FORCE = NORMAL_FORCE * STAT_FRICT_COEF; // the force applied to the bottom of the wheel due to static friction
    constexpr static float WHEEL_TORQUE_FRICT = STAT_FRICT_FORCE * WHEEL_RADIUS; // the torque applied to each wheel due to friction

    constexpr static float CURRENT = 3.6 * 0.5; // I don't know what this value actually is; 3.6 --> stall; 1.15 --> free
    constexpr static float VOLTS = 9.0;
    constexpr static float MAX_POWER = CURRENT * VOLTS; // I don't know what this value actually is
    constexpr static float WHEEL_ANGULAR_SPEED_AT_50_POWER = 5.0; // I don't know what this value actually is; measure this
    constexpr static float MAX_WHEEL_ANGULAR_SPEED = WHEEL_ANGULAR_SPEED_AT_50_POWER * 2.0;
    // constexpr static float motorTorque = (MAX_POWER * motorPowerPercent) / MAX_WHEEL_ANGULAR_SPEED;

    // constexpr static float wheelNetTorque = motorTorque - WHEEL_TORQUE_FRICT;
    // constexpr static float wheelRotAccel = wheelNetTorque / WHEEL_MoI; // the angular acceleration of the wheel
    // constexpr static float WHEEL_ROT_VEL = WHEEL_ROT_ACCEL * deltaTime;
    // constexpr static float WHEEL_ROT_DISPLACEMENT = WHEEL_ROT_ACCEL * deltaTime * deltaTime;
private:
};





#endif
