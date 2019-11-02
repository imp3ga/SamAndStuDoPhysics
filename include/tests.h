#include <Eigen/Core>
#include <string>
#include <iostream>


class UnitTest
{
    public:
        static void RunRests();
    private:
        UnitTest(std::string name);
        void AssertEqual(int expected, int result);
        void AssertEqual(double expected, double result);
        void AssertEqual(Eigen::Vector2d expected, Eigen::Vector2d result);
        
        static void TestVelocityAfterCollision();
        
        std::string _testName;

};