#include <GL/freeglut.h>
#include <iostream>

#include "include/main.h"
// #include "include/solarSystem.h"

void display(){
    glClear(GL_COLOR_BUFFER_BIT);

    std::vector<Eigen::Vector2d> allCentres;
    std::vector<double> allRadii;
    _solarSystem.getInfo(allCentres, allRadii);

    for(int i = 0; i < allCentres.size(); ++i)
    {
        glBegin(GL_POLYGON);
        for(float arc = 0; arc < 2 * CONST_PI; arc += 0.1){
            glVertex2f(allRadii[i]*(cos(arc)) + allCentres[i][0], allRadii[i]*(sin(arc)) + allCentres[i][1]);
        }
        glEnd();
    }
    glFlush();
    glutSwapBuffers();
}

void physicsLoop(int val){
    display();
    _solarSystem.update();
    glutTimerFunc(1, physicsLoop, 0);                                       // 1ms
}

void mouseHandler(int button, int state, int x, int y) 
{
    x -= _dHalfWindowWidth;
    y = - (y - _dHalfWindowHeight);

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
        double r = 100.0 * t * t;
        _solarSystem.addPlanet(pos, v, r);
    }
  }
}

void keyHandler(unsigned char key, int x, int y)
{
    if(key == 'r')
    {
        _solarSystem.reset();
    }
    // else if(key == ' ')
    // {
    //     std::cout << "(" << x << ", " << y << ")\n";
    // }
}

int main(int argc, char **argv){
    _solarSystem.init(50.0);

    // openGL initialization
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE);
    glutInitWindowSize(_dHalfWindowWidth * 2.0, _dHalfWindowHeight * 2.0);
    glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH)- _dHalfWindowWidth * 2.0)/2,
                        (glutGet(GLUT_SCREEN_HEIGHT)- _dHalfWindowHeight * 2.0)/2);
    glutCreateWindow("Planets");
    glOrtho(-_dHalfWindowWidth, _dHalfWindowWidth, -_dHalfWindowHeight, _dHalfWindowHeight, 0, 1);
    glutDisplayFunc(display);
    glutMouseFunc(mouseHandler);
    glutKeyboardFunc(keyHandler);
    physicsLoop(0);

    glutMainLoop();
   
    return 0;
}


