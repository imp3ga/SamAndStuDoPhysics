#include "include/tests.h"
#include "include/solarSystem.h"


UnitTest::UnitTest(std::string name){
    _testName = name;
}

void UnitTest::AssertEqual(double expected, double result){
    std::string testOutput = expected == result ? "PASS\n\n" : "FAIL\n";
    std::cout << "\nTest: " << _testName 
    << "\nExpected: " << expected 
    << "\nResult: " << result 
    << std::endl << testOutput;
}

void UnitTest::AssertEqual(int expected, int result){
    std::string testOutput = expected == result ? "PASS\n\n" : "FAIL\n";
    std::cout << "\nTest: " << _testName 
    << "\nExpected: " << expected 
    << "\nResult: " << result 
    << std::endl << testOutput;
}

void UnitTest::AssertEqual(Eigen::Vector2d expected, Eigen::Vector2d result){
    std::string testOutput = (expected - result).norm() <= 1E-10 ? "PASS\n\n" : "FAIL\n";
    std::cout << "\nTest: " << _testName 
    << "\nExpected: [" << expected[0] << ", " << expected[1] 
    << "]\nResult: [" << result[0] << ", "  << result[1]
    << "]" << std::endl << testOutput;
}

void UnitTest::RunRests(){
    
   UnitTest::TestVelocityAfterCollision();
}

//Test head on collision of two equal objects. Expect basic reversal of velocities.
void UnitTest::TestVelocityAfterCollision(){
     
    // solarSystem _solarSys = solarSystem(10);

    // planet p0 = planet(1, 1,false);
    // planet p1 = planet(1, 1, false);

    // p0.setPosition(Eigen::Vector2d(-1,0));
    // p0.setPosition(Eigen::Vector2d(1,0));

    // p0.setVelocity(Eigen::Vector2d(5,0));
    // p1.setVelocity(Eigen::Vector2d(-5,0));

    // Eigen::Vector2d p0NewVel, p1NewVel;
    // _solarSys.twoBodyCollision(p0,p1, p0NewVel, p1NewVel);
    // UnitTest CollisionVelocityTest_P0 = UnitTest("Collision Velocity Test P0");
    // CollisionVelocityTest_P0.AssertEqual(Eigen::Vector2d(-5,0), p0NewVel);

    // UnitTest CollisionVelocityTest_P1 = UnitTest("Collision Velocity Test P1");
    // CollisionVelocityTest_P1.AssertEqual(Eigen::Vector2d(5,0), p1NewVel);

    
}