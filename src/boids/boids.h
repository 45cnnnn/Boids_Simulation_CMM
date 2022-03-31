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
        FREEFALL=0, SEPARATION=1, ALIGNMENT=2, COHESION=3, LEADER=4
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
    int n;
    bool update = false;
public:
    Boids() :n(1) {}
    Boids(int n) :n(n) {
        initializePositions();
    }
    ~Boids() {}

    void setParticleNumber(int n) {n = n;}
    int getParticleNumber() { return n; }
    void initializePositions()
    {
        positions = VectorXT::Zero(n * dim).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}); 
        velocities = VectorXT::Zero(n * dim);
    }

    void updateBehavior(MethodTypes type)
    {
        if(!update)
            return;
        switch (type)
        {  
        default:
            break;
        }
    }
    void pause()
    {
        update = !update;
    }
    VectorXT getPositions()
    {
        return positions;
    }
};
#endif
