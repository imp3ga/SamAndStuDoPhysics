#include "flowSim/include/basicParticle.h"
#include <functional>

class fluidParticle : public basicParticle
{
    public:
        fluidParticle(Eigen::Vector2d pos);
        fluidParticle(int posX, int posY, int id);
        Eigen::Vector2d getPosition(){return _position;};
        void move(double t, std::function<Eigen::Vector2d(Eigen::Vector2d)> velocityFieldFunction);
        double getLifeTime(){return _dLifeTime;};
        void incrementLifeTime(double dt);   
        void resetLifeTime(); 
    private:

        double _dLifeTime;          
};