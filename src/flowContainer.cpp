

#include "include/flowContainer.h"
#include <GL/freeglut.h>
#include <iostream>


flowContainer::flowContainer(int particleNum, int width, int height)
{
    this->_areaWidth = width;
    this->_areaHeight = height;
    this->_minX = -width / 2;
    this->_maxX = width / 2;
    this->_minY = -height / 2;
    this-> _maxY = height / 2;

    std::cout << "Dimensions: X(" <<  this->_minX << ", " << this->_maxX  << "), Y(" <<  this->_minY << ", " << this->_maxY  << "\n";

    printHelp();

    velocityField::setFieldDimensions(width, height);

    int x, y;
    for(int i = 0; i < particleNum; i++){

        x =  this->_minX + std::rand() * this->_areaWidth;  
        y =  this->_minY + std::rand() * this->_areaHeight;

        _particles.push_back(fluidParticle(x, y, i));
    };
}
void flowContainer::display(){
     glClear(GL_COLOR_BUFFER_BIT);

   
    for(int i = 0; i < this->particleCount(); ++i)
    {
        glBegin(GL_POLYGON);
        
        for(float arc = 0; arc < 2 * CONST_PI; arc += 0.5){
            glVertex2f((cos(arc)) + this->particles()[i].getPosition()[0], (sin(arc)) + this->particles()[i].getPosition()[1]);
        }

        glEnd();
    }
    glFlush();
    glutSwapBuffers();
}

void  flowContainer::update(double t)
{
    
   

    size_t size = this->_particles.size();

    for(int i = 0; i < size; i++){
        
        if(this->particleOutsideBounds(this->particles()[i]) || this->particles()[i].getLifeTime() > CONST_LIFETIME){

            this->resetParticle(this->particles()[i]);
            this->particles()[i].resetLifeTime();
        }
        this->_particles[i].move(t, _velocityFieldFunc);
    };
    
}


bool flowContainer::particleOutsideBounds(basicParticle &p)
{
    return p.getPosition()[0] < this->_minX
    || p.getPosition()[0] > this->_maxX
    || p.getPosition()[1] < this->_minY
    || p.getPosition()[1] > this->_maxY; 
}

void flowContainer::printHelp(){
    std::cout << "\n***** 2D Fluid Sim *****\n";
    std::cout << "\nCONTROLS:\n\n";
    std::cout << " V     --->    Cycle through velocity fields \n";
    std::cout << " R     --->    Reset \n";
    std::cout << " H     --->    Show Help \n";
    std::cout << " M     --->    Toggle Sim Mode \n";
    std::cout << " , / . --->    Flow rate + / - \n";
    std::cout << " P / L --->    Central object radius + / - \n";
    std::cout << " O / K --->    Rankine Vortex rotation + / - \n";
    std::cout << " I / J --->    Cylinder rotation + / - \n\n";
}

void flowContainer::handleKeyPress(unsigned char key){

    if(key == 'v'){
        cycleVelocityField();
    }
    if(key == 'h'){
        printHelp();
    }
    else if(key == 'p' ){
        velocityField::setRadius(50, velocityField::higher);
    }
    else if(key == 'l' ){
        velocityField::setRadius(50, velocityField::lower);
    }
    else if(key == 'o' ){
        velocityField::setRankineVortexRotation(1, velocityField::higher);
    }
    else if(key == 'k' ){
        velocityField::setRankineVortexRotation(1, velocityField::lower);
    }
    else if(key == 'i' ){
        velocityField::setCylinderRotation(10000, velocityField::higher);
    }
    else if(key == 'j' ){
        velocityField::setCylinderRotation(10000, velocityField::lower);
    }
    else if(key == '.'){
        velocityField::setFlowStrength(10, velocityField::higher);
    }
    else if(key == ','){
        velocityField::setFlowStrength(10, velocityField::lower);
    }
    else if(key == 'r'){
        velocityField::resetScalingParameters();
    }
}

///Resets particle position to random point on random wall (top, bottom, left or right)
void flowContainer::resetParticle(basicParticle &p)
{
    int vertical_or_horizontal = std::rand() % 2; //binary choice, reset to side walls (0) or top/bottom walls (1) 
    double newX, newY;

    if(vertical_or_horizontal == 0){ 
        
        int x_Opt = std::rand() % 2;
        
        if(x_Opt == 0){  //binary choice, left(0) or right(1) wall
            newX = this->_minX;
        }
        else{
            newX = this->_maxX;
        }

        newY = this->_minY + std::rand() % this->_areaHeight;
    }
    else{
       
         int y_Opt = std::rand() % 2;
        
        if(y_Opt == 0){ //binary choice, top(0) or bottom(1) wall
            newY = this->_minY;
        }
        else{
            newY = this->_maxY;
        }
        newX = this->_minX + std::rand() % this->_areaWidth;    
    }
    p.setPosition(newX, newY);

}

void flowContainer::setVelocityFieldFunc(std::function<Eigen::Vector2d(Eigen::Vector2d)> func)
{
    this->_velocityFieldFunc = func;
} 

void flowContainer::cycleVelocityField()
{
    switch(_selectedField){
        case noFlow:
            setVelocityFieldFunc(velocityField::uniformFlow);
            _selectedField = uniformFlow;
            std::cout << "Set Uniform Flow" << "\n";
            break;
        case uniformFlow:
            setVelocityFieldFunc(velocityField::shearFlow);
            _selectedField = shearFlow;
            std::cout << "Set Shear Flow" << "\n";
            break;
        case shearFlow:
            setVelocityFieldFunc(velocityField::stagnationPointFlow);
            _selectedField = stagnationPointFlow;
            std::cout << "Set Stagnation Point Flow" << "\n";
            break;
        case stagnationPointFlow:
            setVelocityFieldFunc(velocityField::sphereFlow);
            _selectedField = sphereFlow;
            std::cout << "Set Sphere Flow" << "\n";
            break;
        case sphereFlow:
            setVelocityFieldFunc(velocityField::vortex);
            _selectedField = vortex;
            std::cout << "Set Plain Vortex" << "\n";
            break;
        case vortex:
            setVelocityFieldFunc(velocityField::bathplugVortex);
            _selectedField = bathplugVortex;
            std::cout << "Set Rankine Vortex" << "\n";
            break;
        case bathplugVortex:
            setVelocityFieldFunc(velocityField::liftonCylinder);
            _selectedField = liftonCylinder;
            std::cout << "Set Cylinder Lift Flow" << "\n";
            break;
        case liftonCylinder:
            setVelocityFieldFunc(velocityField::noFlow);
            _selectedField = noFlow;
            std::cout << "Set No Flow" << "\n"; 
            break;
    }
   
}