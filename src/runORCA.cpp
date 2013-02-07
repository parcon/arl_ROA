/* Function to run the ORCA algorithm on the quadrotor. Currently only written to run ORCA between 2-agents. 
    Would need some loops to run more agents as well as more inputs. Possibly a nested function searching through
    neighbors as originally implemented in RVO2_3D


        Inputs: Relative Postion (pos_rel)
                Velocity A       (vel_a)
                Velocity B       (vel_b)
        Outputs: New collision-free velocity (new_v)

    Daman Bareiss
    2/5/2013
    3:05pm
*/

#include "runORCA.h"
/*
#include "Vector3.h" //     arl_RVO2_3D/Vector3.h
#include "Agent.h"
#include <cstddef>
*/
using namespace RVO;
        
/* Search for the best new velocity. */
void runORCA(const Vector3& pos_rel, const Vector3& vel_a, const Vector3& vel_b, const Vector3& cmd_vel, const float& max_vel, Vector3& new_vel)
{
    const float timeStep = 0.02;    // Time step of simulation
    const float timeHorizon_ = 2.0; // Time horizon to check for collision

    // Inverse of time horizon
    const float invTimeHorizon = 1.0f / timeHorizon_;   

    // Relative velocity of two quads
    Vector3 vel_rel = vel_a-vel_b;                  
    
    // Distance between the two quads
    const float distSq = absSq(pos_rel);                

    // Combined radius of the two quads (m)
    const float combinedRadius = 1.2;                   

    // Square of combined radius of quads.
    const float combinedRadiusSq = sqr(combinedRadius); 
        
    // Variable to save half-plane from ORCA
    Plane plane;                                        

    // Vector of half-planes for finding safe velocity for more than 2 agents
    std::vector<Plane> orcaPlanes;                      

    // Unit vector of half-plane
    Vector3 u;                                          
    
    // if the robots will not collide:
    if (distSq > combinedRadiusSq)                      
    {
	    const Vector3 w = vel_rel - invTimeHorizon * pos_rel;
        const float wLengthSq = absSq(w);
		const float dotProduct1 = w * pos_rel;
		if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) 
        {
			const float wLength = std::sqrt(wLengthSq);
			const Vector3 unitW = w / wLength;
			plane.normal = unitW;
			u = (combinedRadius * invTimeHorizon - wLength) * unitW;
		}
		else 
        {
			const float A = distSq;
			const float B = pos_rel * vel_rel;
			const float C = absSq(vel_rel) - absSq(cross(pos_rel, vel_rel)) / (distSq - combinedRadiusSq);
			const float t = (B + std::sqrt(sqr(B) - A * C)) / A;
			const Vector3 w = vel_rel - t * pos_rel;
			const float wLength = abs(w);
			const Vector3 unitW = w / wLength;
			plane.normal = unitW;
			u = (combinedRadius * t - wLength) * unitW;
		}
	}
    // if the robots will collide:
	else 
    {
		const float invTimeStep = 1.0f / timeStep;
		const Vector3 w = vel_rel - invTimeStep * pos_rel;
		const float wLength = abs(w);
		const Vector3 unitW = w / wLength;
		plane.normal = unitW;
		u = (combinedRadius * invTimeStep - wLength) * unitW;
	}
    plane.point = vel_a + 0.5f * u;
	orcaPlanes.push_back(plane);
    const size_t planeFail = linearProgram3(orcaPlanes, max_vel, cmd_vel, false, new_vel);
	if (planeFail < orcaPlanes.size()) {
		linearProgram4(orcaPlanes, planeFail, max_vel, new_vel);
	}
}
 



