#ifndef BOIDS_H
#define BOIDS_H
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Sparse>
template <typename T, int dim>
using Vector = Eigen::Matrix<T, dim, 1, 0, dim, 1>;

template <typename T, int n, int m>
using Matrix = Eigen::Matrix<T, n, m, 0, n, m>;

// add more for yours
enum MethodTypes {
        FREEFALL=0, SEPARATION=1, ALIGNMENT=2, COHESION=3, LEADER=4, CIRCLE=5, COLLISION_AVOIDANCE=6
    };
enum UpdateRule{
    EXPLICIT_EULER, SYMPLECTIC_EULER, EXPLICIT_MIDPOINT
};

using T = double;
using VectorXT = Matrix<T, Eigen::Dynamic, 1>;
using TV = Vector<T, 2>;
using TM = Matrix<T, 2, 2>;

class Boids
{
    
    int dim = 2;
    
private:
    VectorXT positions;
    VectorXT velocities;
    VectorXT accelerations;
    int n;
    bool update = false;
    TV leader_pos;
    /*Hyperparameters*/
    T h = 0.01;                         // update step size
    T radius = 0.5;                      // cohesion circle radius
    T min_dist = 0.05;                   // saparetion circle radius

    T target = -1;                     // target point
    T avoid_dist = 0.5;
    /*Hyperparameters_end*/
public:
    Boids() :n(1) {}
    Boids(int n) :n(n) {
        initializePositions();
    }
    Boids(int n, MethodTypes type) :n(n){
        initializePositions(type);
    }
    ~Boids() {}

    void setParticleNumber(int n) {n = n;}
    int getParticleNumber() { return n; }
    void initializePositions()
    {
        positions = VectorXT::Zero(n * dim).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}); 
        velocities = VectorXT::Zero(n * dim);
        accelerations = VectorXT::Zero(n * dim);;
    }
    void initializePositions(MethodTypes type)
    {
        if(type == CIRCLE){
            positions = VectorXT::Zero(n * dim).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}); 
            velocities = VectorXT::Zero(n * dim);
            for(int i = 0; i < n; i++){
                velocities(2*i) = -positions(2*i+1);
                velocities(2*i+1) = positions(2*i);
            }
            accelerations = VectorXT::Zero(n * dim);;
        }
        //TODO : 初始位置避开障碍物
        else if(type == COLLISION_AVOIDANCE){
        positions = VectorXT::Zero(n * dim).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}) + obs_radius*VectorXT::Ones(n * dim); 
        velocities = VectorXT::Zero(n * dim);
        accelerations = VectorXT::Zero(n * dim);;
        leader_pos = {target,target};  
        }
        else{
        positions = VectorXT::Zero(n * dim).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}); 
        velocities = VectorXT::Zero(n * dim);
        accelerations = VectorXT::Zero(n * dim);;            
        }
    }

    // void updateBehavior(MethodTypes type)
    // {
    //     if(!update)
    //         return;
    //     switch (type)
    //     {  
    //     default:
    //         break;
    //     }
    // }
    void updateBehavior(MethodTypes type, UpdateRule rule);
    void pause()
    {
        update = !update;
    }
    VectorXT getPositions()
    {
        return positions;
    }

    TV getLeaderPos(){
        return leader_pos;
    }
    void updateParam(MethodTypes type, UpdateRule rule);
    // VectorXT updatePos(VectorXT x0, VectorXT v0);
    // VectorXT updateVel(VectorXT v0, VectorXT a0);
    VectorXT updateAcc(VectorXT x0, MethodTypes currentMethod);

    T obs_radius = 0.3;  
};
#endif
