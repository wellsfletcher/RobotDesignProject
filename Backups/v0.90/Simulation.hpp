/*
 
 
 
 SHARED simulation & non-simulation stuff
 
 
 
*/






// include statements

#include "Classes.hpp"
using namespace std;

// important global variable declarations

#define IS_SIMULATION 0
extern Vehicle simulatedVehicle; // this declaration enables the use of the simulatedVehicle object across files






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






/*
 
 
 
 SIMULATION only stuff
 
 
 
 */






// {omitted}