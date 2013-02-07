#include "Vector3.h" //     arl_RVO2_3D/Vector3.h
#include "Agent.cpp"
#include <cstddef>

using namespace RVO;
        
/* Search for the best new velocity. */

void runORCA(const Vector3& pos_rel, const Vector3& vel_a, const Vector3& vel_b, const Vector3& cmd_vel, const float& max_vel, Vector3& new_vel);
