#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <vector>
#include "controller.hpp"

using namespace std;

Controller test;

TEST(ControllerParam, mass)
{
    float mass = 1.5;
    test.set_mass(mass);
    EXPECT_EQ(mass, test.get_mass());
}
TEST(ControllerParam, gravity)
{
    float gravity = 9.81;
    test.set_gravity(gravity);
    EXPECT_EQ(gravity, test.get_gravity());
}
TEST(Controller, J_size)
{
    EXPECT_EQ(3,  test.get_J().rows());
    EXPECT_EQ(3,  test.get_J().cols());
}
TEST(Function, vee_function)
{
    const Matrix3d mat_t = Matrix3d::Random();
    auto vec_t = vee_eigen(mat_t);
    ASSERT_TRUE(vec_t.isApprox(Vector3d{mat_t(2,1), mat_t(0,2), mat_t(1,0)}));
}
TEST(Function, matrix_skew)
{

}
TEST(Function, matrix_inv_skew)
{

}
TEST(Function, hat_function)
{

}
TEST(Controller, attitude)
{

}
TEST(Controller, translation)
{

}
TEST(Controller, position)
{

}


// TEST(TestController, testAttitudeController)
// {
//     Vector3d M_known;
//     M_known << 2.0, 2.0, 3.0;
//
//     Vector3d M;
//     Vector3d W = Vector3d::Zero();
//     Vector3d Wd = Vector3d::Zero();
//     Vector3d Wddot = Vector3d::Zero();
//     Matrix3d R = Matrix3d::Zero();
//     Matrix3d Rd = Matrix3d::Zero();
//
//     test.GeometricPositionController(W,Wd,Wddot, R, Rd, M);
//
//     ASSERT_TRUE(M_known.isApprox(M));
// }
