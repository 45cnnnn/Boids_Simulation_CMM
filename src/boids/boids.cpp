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
        updateParam(type, rule);
        break;
    case ALIGNMENT:
        updateParam(type, rule);
        break;
    case COHESION:
        updateParam(type, rule);
        break;
    case LEADER:
        updateParam(type, rule);
        break;
    case CIRCLE:
        updateParam(type, rule);
        break;
    // case COLLISION_AVOIDANCE:
    //     updateParam(type, rule);
        break;
    default:
        break;
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
    default:
        break;
    }
}
// Assume Mass == 1
VectorXT Boids::updateAcc(VectorXT x0, MethodTypes type){
    
    VectorXT a = VectorXT::Zero(n * dim);

    switch (type)
    {  
    case FREEFALL:
        for(int i = 0; i < n; i++){
            a(2*i) = 0;
            a(2*i+1) = 9.8;
        }
        break;

    case SEPARATION:{
        for(int i = 0; i < n; i++){
            TV neighbour_position = TV::Zero();
            TV neighbour_direction = TV::Zero();
            TV neighbour_separation = TV::Zero();
            int num_cohesion = 0;
            int num_alignment = 0;
            int num_separation = 0;
            for(int j =0; j < n; j++){
                T distance = sqrt((x0(2*i)-x0(2*j))*(x0(2*i)-x0(2*j)) + (x0(2*i+1)-x0(2*j+1))*(x0(2*i+1)-x0(2*j+1)));
                if(distance > 0 && distance <= cohesion_r){
                    neighbour_position(0) += x0(2*j);
                    neighbour_position(1) += x0(2*j+1);
                    num_cohesion += 1;
                }
                if(distance > 0 && distance <= alignment_r){
                    neighbour_direction(0) += velocities(2*j);
                    neighbour_direction(1) += velocities(2*j+1);
                    num_alignment += 1;
                }
                if(distance > 0 && distance <= separation_r){
                    neighbour_separation(0) -= x0(2*j) - x0(2*i);
                    neighbour_separation(1) -= x0(2*j+1) - x0(2*i+1);
                    num_separation += 1;
                }
            } 
            if(num_cohesion != 0){
                a(2*i) = 10*(neighbour_position(0)/num_cohesion - x0(2*i));
                a(2*i+1) = 10*(neighbour_position(1)/num_cohesion - x0(2*i+1)); 
            }
            if(num_alignment != 0){
                a(2*i) += neighbour_direction(0)/num_cohesion - velocities(2*i);
                a(2*i+1) += neighbour_direction(1)/num_cohesion - velocities(2*i+1);
            }
            if(num_separation != 0){
                a(2*i) += 150*neighbour_separation(0)/num_separation;
                a(2*i+1) += 150*neighbour_separation(1)/num_separation;
            }
        }
        break;
    }

    case ALIGNMENT:{
        for(int i = 0; i < n; i++){
            TV neighbour_position = TV::Zero();
            TV neighbour_direction = TV::Zero();
            int num_cohesion = 0;
            int num_alignment = 0;
            for(int j =0; j < n; j++){
                T distance = sqrt((x0(2*i)-x0(2*j))*(x0(2*i)-x0(2*j)) + (x0(2*i+1)-x0(2*j+1))*(x0(2*i+1)-x0(2*j+1)));
                if(distance > 0 && distance <= cohesion_r){
                    neighbour_position(0) += x0(2*j);
                    neighbour_position(1) += x0(2*j+1);
                    num_cohesion += 1;
                }
                if(distance > 0 && distance <= alignment_r){
                    neighbour_direction(0) += velocities(2*j);
                    neighbour_direction(1) += velocities(2*j+1);
                    num_alignment += 1;
                }
            } 
            if(num_cohesion != 0){
                a(2*i) = 10*(neighbour_position(0)/num_cohesion - x0(2*i));
                a(2*i+1) = 10*(neighbour_position(1)/num_cohesion - x0(2*i+1)); 
            }
            if(num_alignment != 0){
                a(2*i) += neighbour_direction(0)/num_cohesion - velocities(2*i);
                a(2*i+1) += neighbour_direction(1)/num_cohesion - velocities(2*i+1);
            }
        }
        break;
    }


    case COHESION:{
        for(int i = 0; i < n; i++){
            TV neighbour = TV::Zero();
            int num_neigh = 0;
            for(int j = 0; j < n; j++){
                T distance = sqrt((x0(2*i)-x0(2*j))*(x0(2*i)-x0(2*j)) + (x0(2*i+1)-x0(2*j+1))*(x0(2*i+1)-x0(2*j+1)));
                if(distance > 0 && distance <= cohesion_r){
                    neighbour(0) += x0(2*j);
                    neighbour(1) += x0(2*j+1);
                    num_neigh += 1;
                }
            }
            if(num_neigh != 0){
                a(2*i) = 10*(neighbour(0)/num_neigh - x0(2*i));
                a(2*i+1) = 10*(neighbour(1)/num_neigh - x0(2*i+1));
            }
            // else{
            //     a(2*i) = 0;
            //     a(2*i+1) = 0;
            // }
            }
        break;
    }  

    case LEADER:
        break;
        
    case CIRCLE:
        a = -x0;
        break;

    // case COLLISION_AVOIDANCE:{
    //     for(int i =0; i < n; i++){
    //         TV target_pos = getLeaderPos();
    //         T distance_obs = sqrt(x0(2*i)*x0(2*i) + x0(2*i+1)*x0(2*i+1));
    //         T distance_target = sqrt((x0(2*i)-target_pos(0))*(x0(2*i)-target_pos(0)) + (x0(2*i+1)-target_pos(1))*(x0(2*i+1)-target_pos(1)));
    //         TV neighbour_position = TV::Zero();
    //         TV neighbour_direction = TV::Zero();
    //         TV neighbour_separation = TV::Zero();
    //         int num_cohesion = 0;
    //         int num_separation = 0;
    //         for(int j =0; j < n; j++){
    //             T distance = sqrt((x0(2*i)-x0(2*j))*(x0(2*i)-x0(2*j)) + (x0(2*i+1)-x0(2*j+1))*(x0(2*i+1)-x0(2*j+1)));
    //             if(distance > 0 && distance <= cohesion_r){
    //                 neighbour_position(0) += x0(2*j);
    //                 neighbour_position(1) += x0(2*j+1);
    //                 neighbour_direction(0) += velocities(2*j);
    //                 neighbour_direction(1) += velocities(2*j+1);
    //                 num_cohesion += 1;
    //             }
    //             if(distance > 0 && distance <= separation_r){
    //                 neighbour_separation(0) -= x0(2*j) - x0(2*i);
    //                 neighbour_separation(1) -= x0(2*j+1) - x0(2*i+1);
    //                 num_separation += 1;
    //             }

    //         } 
    //         if(num_cohesion != 0){
    //             a(2*i) = 10*(neighbour_position(0)/num_cohesion - x0(2*i));
    //             a(2*i+1) = 10*(neighbour_position(1)/num_cohesion - x0(2*i+1)); 
    //             a(2*i) += neighbour_direction(0)/num_cohesion - velocities(2*i);
    //             a(2*i+1) += neighbour_direction(1)/num_cohesion - velocities(2*i+1);
    //         }
    //         if(num_separation != 0){
    //             a(2*i) += 150*neighbour_separation(0)/num_separation;
    //             a(2*i+1) += 150*neighbour_separation(1)/num_separation;
    //         }

    //         if(distance_obs <= obs_radius + sight){
    //         a(2*i) +=  10*x0(2*i)/distance_obs;
    //         a(2*i+1) +=  10*x0(2*i+1)/distance_obs;
    //         }
    //         a(2*i) += 5*(target_pos(0) - x0(2*i))/distance_target - velocities(2*i);
    //         a(2*i+1) += 5*(target_pos(1) - x0(2*i+1))/distance_target - velocities(2*i+1);
    //     }
    //     break;
    // }
    default:
        break;
    }
    if(obs_flag){
        for(int i = 0; i < n; i++){
            T distance_obs = sqrt((x0(2*i)-obs_pos(0))*(x0(2*i)-obs_pos(0)) + (x0(2*i+1)-obs_pos(1))*(x0(2*i+1)-obs_pos(1)));
            if(distance_obs > obs_radius && distance_obs <= obs_radius + sight){
                a(2*i) +=  5*(x0(2*i) - obs_pos(0))/pow(distance_obs, 2);
                a(2*i+1) +=  5*(x0(2*i+1) - obs_pos(1))/pow(distance_obs,2); 
            }
            else if(distance_obs <= obs_radius){
                a(2*i) +=  20*(x0(2*i) - obs_pos(0));
                a(2*i+1) +=  20*(x0(2*i+1) - obs_pos(1));                 
            }
        }
    }
    // if(currentMethod == FREEFALL){
    //     for(int i = 0; i < n; i++){
    //         a(2*i) = 0;
    //         a(2*i+1) = 9.8;
    //     }
    // }
    // if(currentMethod == CIRCLE){
    //     a = -x0;
    // }
    // if(currentMethod == COHESION){
    //     for(int i = 0; i < n; i++){
    //         TV neighbour = TV::Zero();
    //         int num_neigh = 0;
    //         for(int j = 0; j < n; j++){
    //             T distance = sqrt((x0(2*i)-x0(2*j))*(x0(2*i)-x0(2*j)) + (x0(2*i+1)-x0(2*j+1))*(x0(2*i+1)-x0(2*j+1)));
    //             if(distance > 0 && distance <= radius){
    //                 neighbour(0) += x0(2*j);
    //                 neighbour(1) += x0(2*j+1);
    //                 num_neigh += 1;
    //             }
    //         }
    //         if(num_neigh != 0){
    //             a(2*i) = (neighbour(0)/num_neigh - x0(2*i));
    //             a(2*i+1) = (neighbour(1)/num_neigh - x0(2*i+1));
    //         }
    //         // else{
    //         //     a(2*i) = 0;
    //         //     a(2*i+1) = 0;
    //         // }
    //         }
    // }
    return a;
}

