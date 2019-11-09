
#include <Eigen/Core>

class velocityField
{
    public:

        //velocityField(int w, int h);
        //velocityField(int w, int h, double flow);
        //velocityField(int w, int h, double flow, double radius, double rankineRotation, double cylinderRotation);

        static Eigen::Vector2d noFlow(const Eigen::Vector2d& position);    
        static Eigen::Vector2d uniformFlow(const Eigen::Vector2d& position);    
        static Eigen::Vector2d shearFlow(const Eigen::Vector2d& position);    
        static Eigen::Vector2d stagnationPointFlow(const Eigen::Vector2d& position);    
        static Eigen::Vector2d sphereFlow(const Eigen::Vector2d& position);    
        static Eigen::Vector2d vortex(const Eigen::Vector2d& position);    
        static Eigen::Vector2d bathplugVortex(const Eigen::Vector2d& position);    
        static Eigen::Vector2d liftonCylinder(const Eigen::Vector2d& position);

        static void setFieldDimensions(int w, int h);
        static void setFlowStrength(double strength, int higherLower);    
        static void setRadius(double radius, int higherLower);    
        static void setCylinderRotation(double rotation, int higherLower);    
        static void setRankineVortexRotation(double rotation, int higherLower);  
        static void resetScalingParameters();

        static double getRadius(){return _dRadius;}; 
        static double getFlow(){return _dFlowStrength;}; 
        static double getCylinderRotation(){return _dCylinderrotation;}; 
        static double getRankineRotation(){return _dRankinevortexrotation;}; 



        enum incrementDirecton{
            higher,
            lower
        };
    private:

        static int _areaWidth;
        static int _areaHeight;
        static double _dFlowStrength;
        static double _dRadius;
        static double _dCylinderrotation;
        static double _dRankinevortexrotation;
        static std::out_of_range _outOfRangeError;
};
