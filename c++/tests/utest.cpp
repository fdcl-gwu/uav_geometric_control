#include "gtest/gtest.h"
#include <vector>
#include "controller.hpp"

controller test;

TEST(Controller, position)
{
    float mass = 1.5;
    test.set_mass(mass);
    EXPECT_EQ(mass, test.get_mass());
}
