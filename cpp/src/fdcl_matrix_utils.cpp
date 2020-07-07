#include "fdcl/matrix_utils.hpp"


Matrix3 hat(const Vector3 v)
{
    Matrix3 V;
    V.setZero();

    V(2,1) = v(0);
    V(1,2) = -V(2, 1);
    V(0,2) = v(1);
    V(2,0) = -V(0, 2);
    V(1,0) = v(2);
    V(0,1) = -V(1, 0);

    return V;
}


Vector3 vee(const Matrix3 V)
{
    // TODO: improve code by: https://codereview.stackexchange.com/questions/77546/multiply-vector-elements-by-a-scalar-value-using-stl-and-templates
    Vector3 v;
    Matrix3 E;

    v.setZero();
    E = V + V.transpose();
    
    if(E.norm() > 1.e-6)
    {
        std::cout << "VEE: E.norm() = " << E.norm() << std::endl;
    }

    v(0) = V(2, 1);
    v(1) = V(0, 2);
    v(2) = V(1, 0);

    return  v;
}


void saturate(Vector3 &x, const double x_min, const double x_max)
{
    for (int i = 0; i < 3; i++)
    {
        if (x(i) > x_max) x(i) = x_max;
        else if (x(i) < x_min) x(i) = x_min;
    }
}

