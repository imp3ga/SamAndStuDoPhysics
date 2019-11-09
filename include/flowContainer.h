#include "include/fluidParticle.h"
#include "include/velocityField.h"
#include <vector>
#define CONST_PI 3.14159265358979
#define CONST_LIFETIME 4

class flowContainer
{
    public:
        //Constructor
        flowContainer(int particleNum, int width, int height);
        
        //Setters
        void setVelocityFieldFunc(std::function<Eigen::Vector2d(Eigen::Vector2d)> func);       
        
        //Methods
        void update(double t);
        int particleCount(){return _particles.size();};
        std::vector<fluidParticle> &particles(){return _particles;}
        void resetParticle(basicParticle &p);
        bool particleOutsideBounds(basicParticle &p);
        void display();
      
        void handleKeyPress(unsigned char key);
        void printHelp();

    private:
        int _areaWidth;
        int _areaHeight;
        int _minX, _maxX, _minY, _maxY;             // Axes for velocityField are set to centre of screen, so need to convert _areaWidth and _areaHeight
                                                    // i.e. if _areaWidth == 480 ----> (minX, maxX) = (-240, 240)#
                                                    // This is managed in constructor.
        std::vector<fluidParticle> _particles;
        std::function<Eigen::Vector2d(Eigen::Vector2d)> _velocityFieldFunc = velocityField::uniformFlow;
        int _selectedField = 0;
        void cycleVelocityField();
        enum selected_field
        {
            noFlow,
            uniformFlow,
            shearFlow,
            stagnationPointFlow,
            sphereFlow,
            vortex,
            bathplugVortex,
            liftonCylinder
        };
};