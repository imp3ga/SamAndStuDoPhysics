#include <Eigen/Core>
#include "include/astroObjectBase.h"

#define CONST_PI 3.14159265358979

class planet : public AstroObjectBase
{
    public:
        planet(const double r, const double rho, bool fixed);
        void setRadius(const double r);

        double getRadius(){return dRadius;};

    private:
        double dRadius;
};