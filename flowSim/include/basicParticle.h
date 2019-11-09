#include <Eigen/Core>
#include <ctime>

class basicParticle
{
    public:
        int _id; // for better debugging visuals (only print for id==0, instead of all 2000)
        void setPosition(Eigen::Vector2d pos);
        void setPosition(int x, int y);
        void setVelocity(Eigen::Vector2d vel);
        void move(double t);
        Eigen::Vector2d getPosition(){return _position;};

    protected:
        Eigen::Vector2d _position;
        Eigen::Vector2d _velocity;
};