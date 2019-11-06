#include <GL/freeglut.h>
#include <iostream>
#include <iomanip>

#include "include/main.h"
#include "include/planet.h"
// #include "include/solarSystem.h"

// void display(){
//     glClear(GL_COLOR_BUFFER_BIT);

//     std::vector<Eigen::Vector2d> allCentres;
//     std::vector<double> allRadii;
//     _solarSystem.getInfo(allCentres, allRadii);

//     for(int i = 0; i < allCentres.size(); ++i)
//     {
//         glBegin(GL_POLYGON);
//         for(float arc = 0; arc < 2 * CONST_PI; arc += 0.5){
//             glVertex2f(allRadii[i]*(cos(arc)) + allCentres[i][0], allRadii[i]*(sin(arc)) + allCentres[i][1]);
//         }
//         glEnd();
//     }
//     glFlush();
//     glutSwapBuffers();
// }

// void physicsLoop(int val){
//     display();
//     _solarSystem.update();
//     glutTimerFunc(1, physicsLoop, 0);                                       // 1ms
// }

// void mouseHandler(int button, int state, int x, int y) 
// {
//     x -= _dHalfWindowWidth;
//     y = - (y - _dHalfWindowHeight);

//   if (state == GLUT_DOWN) {
//     if (button == GLUT_LEFT_BUTTON) {
//         _initXY = Eigen::Vector2d(x, y);
//         _initTime = Clock::now();
//     }
//   }

//   if (state == GLUT_UP) {
//     if (button == GLUT_LEFT_BUTTON) {
//         Eigen::Vector2d pos(x, y);
//         Eigen::Vector2d mouseDisp = pos - _initXY;
//         double t = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - _initTime).count()  / 1E9;
//         Eigen::Vector2d v = 2.0 * mouseDisp / t;
//         double r = 100.0 * t * t;
//         _solarSystem.addPlanet(pos, v, r);
//     }
//   }
// }

// void keyHandler(unsigned char key, int x, int y)
// {
//     if(key == 'r')
//     {
//         // reset
//         _solarSystem.reset();
//     }
//     else if(key == 'c')
//     {   
//         // centre
//         _solarSystem.centre();
//     }
//     // else if(key == ' ')
//     // {
//     //     std::cout << "(" << x << ", " << y << ")\n";
//     // }
// }


// void runSolarSystem(int argc, char **argv){
//     _solarSystem.init(50.0);

//     // openGL initialization
//     glutInit(&argc, argv);
//     glutInitDisplayMode(GLUT_DOUBLE);
//     glutInitWindowSize(_dHalfWindowWidth * 2.0, _dHalfWindowHeight * 2.0);
//     glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH)- _dHalfWindowWidth * 2.0)/2,
//                         (glutGet(GLUT_SCREEN_HEIGHT)- _dHalfWindowHeight * 2.0)/2);
//     glutCreateWindow("Planets");
//     glOrtho(-_dHalfWindowWidth, _dHalfWindowWidth, -_dHalfWindowHeight, _dHalfWindowHeight, 0, 1);
//     glutDisplayFunc(display);
//     glutMouseFunc(mouseHandler);
//     glutKeyboardFunc(keyHandler);
//     physicsLoop(0);

//     glutMainLoop();
// }

int main(int argc, char **argv){

    std::vector<AstroObjectBase> testVec;
    std::cout << "v located at " << &testVec << std::endl;
    planet testPlanet(0, 100., Eigen::Vector2d(1000., 0.), Eigen::Vector2d(0., 0.), 1., 1000., testVec);
    testVec.push_back(testPlanet);
    planet testPlanet2(0, 100., Eigen::Vector2d(0., 0.), Eigen::Vector2d(0., 0.), 1., 1000., testVec);
    testVec.push_back(testPlanet);
    planet testPlanet3(0, 100., Eigen::Vector2d(0., 1000.), Eigen::Vector2d(0., 0.), 1., 1000., testVec);



    // std::cout << std::setprecision(30) << "Begin!" << std::endl;
    // // std::cout << argv[1];

    // if(argc > 1){
        
    //     std::string argString = argv[1];
        
    //     if(argString == "tests"){
    //         UnitTest::RunRests();
    //     }

    // }
    // else{
    //     runSolarSystem(argc, argv);
    // }

    return 0;
}


