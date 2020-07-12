/*
 * Copyright (c) 2020 Flight Dynamics and Control Lab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#include "fdcl/matrix_utils.hpp"
#include "gtest/gtest.h"


TEST(TestHatAndVeeMap, HatMapZero)
{
    Vector3 x(Vector3::Random()), y(Vector3::Random());
    Matrix3 xhat;
    xhat = hat(x);
    ASSERT_LT((xhat * x).norm(), 1.0e-6);
}


TEST(TestHatAndVeeMap, HatMapSkewSymmetric)
{
    Vector3 x(Vector3::Random()), y(Vector3::Random());
    Matrix3 xhat;
    xhat = hat(x);
    ASSERT_TRUE(xhat.transpose().isApprox(-xhat));
}


TEST(TestHatAndVeeMap, HatMapInverseIsVeeMap)
{
    Vector3 x(Vector3::Random());
    Vector3 xvee;
    Matrix3 xhat;

    xhat = hat(x);
    xvee = vee(xhat);

    ASSERT_TRUE(xvee.isApprox(x));
}


TEST(TestHatAndVeeMap, HatMapCrossProduct)
{
    Vector3 x(Vector3::Random()), y(Vector3::Random());
    Matrix3 xhat, yhat;
    xhat = hat(x);
    yhat = hat(y);
    ASSERT_TRUE((xhat * y).isApprox(x.cross(y)));
    ASSERT_TRUE((xhat * y).isApprox(- yhat * x));
}


TEST(TestSaturate, SaturateLower)
{
    Vector3 x(-2.5, -3.0, -4.0);

    double min_x = -2.0;
    double max_x = 2.0;

    Vector3 sat_x(min_x, min_x, min_x);
    
    saturate(x, min_x, max_x);

    ASSERT_TRUE(x.isApprox(sat_x));
}


TEST(TestSaturate, SaturateUpper)
{
    Vector3 x(2.5, 3.0, 4.0);

    double min_x = -2.0;
    double max_x = 2.0;

    Vector3 sat_x(max_x, max_x, max_x);

    saturate(x, min_x, max_x);

    ASSERT_TRUE(x.isApprox(sat_x));
}


TEST(TestSaturate, SaturateMid)
{
    Vector3 x(-1.5, 0.0, 1.9);

    double min_x = -2.0;
    double max_x = 2.0;

    Vector3 sat_x(x(0), x(1), x(2));

    saturate(x, min_x, max_x);

    ASSERT_TRUE(x.isApprox(sat_x));
}