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
        FREEFALL=0, SEPARATION=1, ALIGNMENT=2, COHESION=3, LEADER=4, CIRCLE=5, CA=6
    };
enum UpdateRule{
    EXPLICIT_EULER=0, SYMPLECTIC_EULER=1, EXPLICIT_MIDPOINT=2
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
        positions = VectorXT::Zero(n * dim).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}) - 0.5*VectorXT::Ones(n*dim); 
        velocities = VectorXT::Zero(n * dim);
        accelerations = VectorXT::Zero(n * dim);;
    }
    void initializePositions(MethodTypes type)
    {   
        // Position initiallization
        if(obs_flag){
            positions = VectorXT::Zero(n * dim).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}) - 0.5*VectorXT::Ones(n * dim);
            for(int i = 0; i < n; i++){
                if(sqrt(pow(positions(2*i), 2) + pow(positions(2*i+1), 2)) <= obs_radius){
                    if(positions(2*i+1) >= 0){positions(2*i+1) += obs_radius;}
                    else{positions(2*i+1) -= obs_radius;}
                }
            }
        }
        else{
            positions = VectorXT::Zero(n * dim).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}) - 0.5*VectorXT::Ones(n*dim); 
     
        }

        // Velocity initiallization
        if(type == CIRCLE){
            velocities = VectorXT::Zero(n * dim);
            for(int i = 0; i < n; i++){
                velocities(2*i) = -positions(2*i+1);
                velocities(2*i+1) = positions(2*i);
            }
        }
        else{
        velocities = VectorXT::Zero(n * dim);            
        }

        //Groups
        if(type == CA){
            for(int i = 0; i < n; i++){
                red_pos(i) = positions(i);
            }
            for(int i = n; i < n*dim; i++){
                blue_pos(i-n) = positions(i);
            }
            red_vel = VectorXT::Zero(n);
            blue_vel = VectorXT::Zero(n);
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
    VectorXT getRedPos(){return red_pos;}
    VectorXT getBluePos(){return blue_pos;}
    int getRedNum(){return red_num;}
    int getBlueNum(){return blue_num;}

    TV getLeaderPos(){
        return leader_pos;
    }
    TV getLeaderVel(){
        return leader_vel;
    }
    TV getObsPos(){
        return obs_pos;
    }
    void updateParam(MethodTypes type, UpdateRule rule);
    // void updateParamCA(MethodTypes type, UpdateRule rule);
    // VectorXT updatePos(VectorXT x0, VectorXT v0);
    // VectorXT updateVel(VectorXT v0, VectorXT a0);
    VectorXT updateAcc(VectorXT x0, MethodTypes currentMethod);
    VectorXT updateAccCA(VectorXT x0, VectorXT v0);

    void creatBoid(TV x0, bool group_flag);
    void removeBoid();

    /*Hyperparameters*/
    float cohesion_r = 0.5;                      // cohesion range radius
    float separation_r = 0.1;                   // saparetion ramge radius
    float alignment_r = 0.5;                     // aligment range radius

    float target = -1;                           // target point
    float sight = 0.55;
    float avoid_distance = 0.1;

    bool obs_flag = 0;
    float obs_radius = 0.1; 
    float h = 0.01;

    float creat_r = 0.1;
    float remove_r = 0.1;
    /*Hyperparameters_end*/

    TV leader_pos = TV::Zero();
    TV leader_vel = TV::Zero();
    TV obs_pos = TV::Zero();
    
    bool group_flag;
    int red_num = n/2;
    int blue_num = n/2;

    VectorXT red_pos = VectorXT::Zero(n);
    VectorXT red_vel;
    VectorXT red_acc;
    VectorXT blue_pos =VectorXT::Zero(n);
    VectorXT blue_vel;
    VectorXT blue_acc;

};
#endif
