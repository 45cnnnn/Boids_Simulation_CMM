#include "boids.h"

void Boids::updateBehavior(MethodTypes type, UpdateRule rule)
{
    if(!update)
        return;
    switch (type)
    {  
    case FREEFALL:
        updateParam(type, rule);
        break;
    case SEPARATION:
        break;
    case ALIGNMENT:
        break;
    case COHESION:
        break;
    case LEADER:
        break;
    // default:
    //     break;
    }
}
//TODO:Ex2 
void Boids::updateParam(MethodTypes type, UpdateRule rule){
    switch (rule)
    {
    case EXPLICIT_EULER:{
        VectorXT last_positions = positions;
        positions = last_positions + h * velocities;
        accelerations = updateAcc(last_positions, type);
        velocities = velocities + h * accelerations;
        break;
    }
    case SYMPLECTIC_EULER:
        positions = positions + h * velocities;
        accelerations = updateAcc(positions, type);
        velocities = velocities + h * accelerations;
        break;
    
    case EXPLICIT_MIDPOINT:{
        VectorXT last_positions = positions;
        VectorXT last_velocities = velocities;
        VectorXT half_position = last_positions + 0.5 * h * velocities;
        accelerations = updateAcc(last_positions, type);
        VectorXT half_velocities = last_velocities + 0.5 * h * accelerations;
        positions = last_positions + h * half_velocities;
        accelerations = updateAcc(half_position, type);
        velocities = last_velocities + h * accelerations;
        break;      
    }     
    // default:
    //     break;
    }
}

VectorXT Boids::updateAcc(VectorXT x0, MethodTypes currentMethod){
    
    VectorXT a = VectorXT::Zero(n * dim);
    if(currentMethod == FREEFALL){
        for(int i = 0; i < n; i++){
            a(2*i) = 0;
            a(2*i+1) = 9.8;
        }
    }
    return a;
}

