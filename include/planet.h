#include <Eigen/Core>

#define CONST_PI 3.14159265358979

class planet
{
    public:
        planet(const double r, const double rho);
        void setRadius(const double r);
        void setMass(const double m);
        void setPosition(const Eigen::Vector2d s);
        void setVelocity(const Eigen::Vector2d v);

        double getRadius(){return dRadius;};
        double getMass(){return dMass;};
        Eigen::Vector2d getPosition(){return position;};
        Eigen::Vector2d getVelocity(){return velocity;};

    private:
        double dRadius, dMass;
        Eigen::Vector2d position, velocity;
};