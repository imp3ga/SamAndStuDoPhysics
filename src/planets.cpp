#include <GL/freeglut.h>
#include <cmath>
#include <iostream>

#include "include/planets.h"

planet::planet(double r)
{
    setRadius(r);
    setMass(r * r * _dMassDensity);
}

void planet::setRadius(double r)
{
    dRadius = r;
}

void planet::setMass(double m)
{
    dMass = m;
}

void planet::setPosition(Eigen::Vector2d s)
{
    position = s;
}

void planet::setVelocity(Eigen::Vector2d v)
{
    velocity = v;
}

void display(){
    glClear(GL_COLOR_BUFFER_BIT); // blit blank display

    
    const double pi = 3.141593;

    for(int i = 0; i < solarSystem.size(); ++i)
    {
        glBegin(GL_POLYGON);
        planet &planet = solarSystem[i];
        Eigen::Vector2d pos = planet.getPosition();
        double dRad = planet.getRadius();
        for(float arc = 0; arc < 2*pi; arc+=0.5){

            glVertex2f(dRad*(cos(arc)) + pos[0], dRad*(sin(arc)) + pos[1]);
        }
        glEnd();
    }

    glFlush();
    glutSwapBuffers();
}

void physicsLoop(int val){
    display();

    // Calculate forces on each object and check for collisions
    const int nObjects = solarSystem.size();
    Eigen::Vector2d aForces[nObjects - 1];
    std::vector<planet> newPlanets;
    for (int i = 0; i < nObjects; ++i)
    {
        planet &p0 = solarSystem[i];
        Eigen::Vector2d p0Pos = p0.getPosition();
        double p0Mass = p0.getMass();
        if(p0Mass < 0)
        {
            continue;
        }
        Eigen::Vector2d force(0., 0.);

        for (int j = 0; j < nObjects; ++j)
        {
            if(i == j)
            {
                continue;
            }
            planet &p1 = solarSystem[j];
            Eigen::Vector2d p1Pos = p1.getPosition();
            double p1Mass = p1.getMass();
            if(p1Mass < 0.)
            { 
                continue;
            }
            // Check for collisions
            double dist = (p1Pos - p0Pos).norm();
            // std::cout << dist << "\n";
            if (dist < p0.getRadius() + p1.getRadius())
            {
                double newMass = p0Mass + p1Mass;
                double newRadius = sqrt(newMass / _dMassDensity);
                // std::cout << "prev masses: " << p0Mass << ", " << p1Mass << ",   new mass: " << newMass << "\n";
                // std::cout << "prev radii: " << p0.getRadius() << ", " << p1.getRadius() << ",   new radius: " << newRadius << "\n";
                // std::cout << "dist was: " << dist << "\n\n";
                // std::cout << "i, j: " << i << ", " << j << "\n\n";
                planet pNew(newRadius);

                pNew.setPosition(p0Mass > p1Mass ? p0.getPosition() : p1.getPosition());
                Eigen::Vector2d p0Vel = p0.getVelocity();
                Eigen::Vector2d p1Vel = p1.getVelocity();
                Eigen::Vector2d newVel = (p0Mass * p0Vel + p1Mass * p1Vel) / newMass;
                pNew.setVelocity(newVel);
                newPlanets.push_back(pNew);
                solarSystem[i].setMass(-1.0);
                solarSystem[j].setMass(-1.0);
                continue;
            }
            double dForce = p0Mass * p1Mass / (p1Pos - p0Pos).squaredNorm();

            force += dForce * (p1Pos - p0Pos);
        }
        aForces[i] = force;
        // std::cout << "Total force on object " << i << " is " << force.transpose() << "\n";
    }


        // Remove negative mass planets, add new ones
        for (int i = 0; i < nObjects; ++i)
        {
            planet &planet = solarSystem[i];
            double m = planet.getMass();
            if(m < 0.0)
            {
                solarSystem.erase(solarSystem.begin() + i);
                ++i;
            }
        }
        solarSystem.insert( solarSystem.end(), newPlanets.begin(), newPlanets.end() );

    // std::cout << "FINISHED check \n";

    for (int i = 0; i < nObjects; ++i)
    {
        planet &planet = solarSystem[i];
        // Update velocity
        double mass = planet.getMass();
        Eigen::Vector2d acc = aForces[i] / mass;
        Eigen::Vector2d newVel = planet.getVelocity() + (0.001 * acc);
        planet.setVelocity(newVel);
        // Update position
        Eigen::Vector2d newPos = planet.getPosition() + (0.001 * newVel);
        planet.setPosition(newPos);
    }

    glutTimerFunc(1, physicsLoop, 0);
}



void mouseHandler(int button, int state, int x, int y) 
{
    x -= _dHalfWindowWidth;
    y -= _dHalfWindowHeight;

  if (state == GLUT_DOWN) {
    if (button == GLUT_LEFT_BUTTON) {
        _initXY = Eigen::Vector2d(x, y);
        _initTime = Clock::now();
    }
  }

  if (state == GLUT_UP) {
    if (button == GLUT_LEFT_BUTTON) {
        Eigen::Vector2d pos(x, y);
        Eigen::Vector2d mouseDisp = pos - _initXY;
        double t = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - _initTime).count()  / 1E9;
        Eigen::Vector2d v = 2.0 * mouseDisp / t;
    
        planet newPlanet(500.0 * t * t);
        newPlanet.setPosition(pos);
        newPlanet.setVelocity(v);
        solarSystem.push_back(newPlanet);
    }
  }
}

int main(int argc, char **argv){
    planet initPlanet(50);
    initPlanet.setPosition(Eigen::Vector2d(0, 0));
    initPlanet.setVelocity(Eigen::Vector2d(0.,0.));
    solarSystem.push_back(initPlanet);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE);
    glutInitWindowSize(_dHalfWindowWidth * 2.0, _dHalfWindowHeight * 2.0);
    glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH)- _dHalfWindowWidth * 2.0)/2,
                        (glutGet(GLUT_SCREEN_HEIGHT)- _dHalfWindowHeight * 2.0)/2);
    glutCreateWindow("Planets");
    glOrtho(-_dHalfWindowWidth, _dHalfWindowWidth, _dHalfWindowHeight, -_dHalfWindowHeight, 0, 1);
    glutDisplayFunc(display);
    glutMouseFunc(mouseHandler);
    physicsLoop(0);

    glutMainLoop();
   

    return 0;
}


