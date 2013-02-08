#include "runORCA.h"
#include <vector>
#include <iostream>
#include <fstream>

// Tester function for runORCA. Run two agents with no dynamics straight at each other and see how they dodge.
int main()
{
    // Time parameters
    float timeStep = 0.02; // Simulation time step
    float simTime  = 10;   // Simulation time
    float maxVel = 2;
    
    // State parameters
    std::vector<Vector3> posA, posB;   // Position of A, Position of B
    std::vector<Vector3> velA, velB;   // Velocity of A, Velocity of B
    std::vector<Vector3> posAB, posBA; // posA-posB, posB-posA
    Vector3 newVel; // Velocity that will come out of ORCA

    // Agent A initialized at position (0,0,0) with velocity (0,0,0) and a goal velocity of (1,0,0)
    Vector3 curPos(0,0,0);
    Vector3 curVel(0,0,0);
    Vector3 AgoalVel(1,0,0);
    posA.push_back(curPos);
    velA.push_back(curVel);
    
    // Agent B initialized at position (10,0,0) with a velocity (0,0,0) and a goal velocity of (-1,0,0)
    curPos[0] = 10; curPos[1] = 0; curPos[2] = 0;
    Vector3 BgoalVel(-1,0,0);
    posB.push_back(curPos);
    velB.push_back(curVel);

    // Relative position found
    posAB.push_back(posA[0]-posB[0]);
    posBA.push_back(posB[0]-posA[0]);

    // Loop through time
    for(int i=0; i<simTime/timeStep; i++)
    {
        /* For quad A:
                Pass desired velocity into ORCA
                Get new goal velocity
           For quad B: 
                Pass desired velocity into ORCA 
                Get new goal velocity
            For quad A:
                Update next position as PA_k+1 = PA_k + VA*delta_T
            For quad B:
                Update next posiiton as PB_k+1 = PB_k + VB*delta_T
        */

        // run ORCA for A->B
        runORCA(posAB[i],velA[i],velB[i],AgoalVel, maxVel, newVel);
        velA.push_back(newVel);
        // run ORCA for B->A
        //runORCA(posBA[i],velB[i],velA[i],BgoalVel, maxVel, newVel);
        velB.push_back(newVel);
        // Update position of A
        posA.push_back(posA[i] + timeStep * velA[i+1]);
        // Update position of B
        posB.push_back(posB[i] + timeStep * velB[i+1]);
    }

    return 0;
}
