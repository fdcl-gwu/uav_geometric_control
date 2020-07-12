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


#include "fdcl/control.hpp"
#include "gtest/gtest.h"

TEST(TestControl, Initialize)
{
    fdcl::state_t *state;
    fdcl::command_t *command;
    fdcl::param *config_file;

    state = new fdcl::state_t();
    command = new fdcl::command_t();

    config_file = new fdcl::param();
    config_file->open("../tests/test_decoupled.cfg");

    fdcl::control controller(state, command, config_file);

    double m = 0;
    Matrix3 J = Matrix3::Identity();
    controller.output_uav_properties(m, J);

    ASSERT_FLOAT_EQ(m, 1.95);

    Matrix3 J_cfg;
    J_cfg << 0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.04;
    ASSERT_TRUE(J.isApprox(J_cfg));

    config_file->close();

    delete config_file;
    delete state;
    delete command;
}


TEST(TestControl, DecoupledZeroInitConditions)
{
    fdcl::state_t *state;
    fdcl::command_t *command;
    fdcl::param *config_file;

    state = new fdcl::state_t();
    command = new fdcl::command_t();

    config_file = new fdcl::param();
    config_file->open("../tests/test_decoupled.cfg");

    double f, f_out;
    Vector3 M, M_out;

    fdcl::control controller(state, command, config_file);

    state->x.setZero();
    state->v.setZero();
    state->a.setZero();
    state->R.setIdentity();
    state->W.setZero();

    command->xd.setZero();
    command->xd_dot.setZero();
    command->xd_2dot.setZero();
    command->xd_3dot.setZero();
    command->xd_4dot.setZero();
    command->b1d << 1.0, 0.0, 0.0;
    command->b1d_dot.setZero();
    command->b1d_ddot.setZero();

    f = 19.1295;
    M << 0.0, 0.0, 0.0;

    controller.position_control();
    controller.output_fM(f_out, M_out);

    ASSERT_FLOAT_EQ(f, f_out);
    ASSERT_TRUE(M.isApprox(M_out));

    config_file->close();

    delete config_file;
    delete state;
    delete command;
}


TEST(TestControl, DecoupledNonZeroInitConditions)
{
    fdcl::state_t *state;
    fdcl::command_t *command;
    fdcl::param *config_file;

    state = new fdcl::state_t();
    command = new fdcl::command_t();

    config_file = new fdcl::param();
    config_file->open("../tests/test_decoupled.cfg");

    double f, f_out;
    Vector3 M, M_out;

    fdcl::control controller(state, command, config_file);

    state->x << 0.0, 1.0, 2.2;
    state->v << 0.0, 1.0, 2.2;
    state->a.setZero();
    state->R << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
    state->W.setZero();

    command->xd << -1.0, 0.0, 0.0;
    command->xd_dot << 0.0, -2.0, 0.0;
    command->xd_2dot << 0.0, 0.0, -5.0;
    command->xd_3dot << 0.0, 1.2, 0.0;
    command->xd_4dot << 2.1, 0.0, 0.0;
    command->b1d << 0.707106781186548, 0.707106781186548, 0.0;
    command->b1d_dot << 0.0, 0.707106781186548, 0.707106781186548;
    command->b1d_ddot << 0.707106781186548, 0.0, 0.707106781186548;

    f = 68.4795;
    M << 1.07467244713611, -0.277520557244241, -0.554472450210804;

    controller.position_control();
    controller.output_fM(f_out, M_out);
    
    // std::cout << M_out << std::endl;

    ASSERT_FLOAT_EQ(f, f_out);
    ASSERT_TRUE(M.isApprox(M_out));

    config_file->close();

    delete config_file;
    delete state;
    delete command;
}


TEST(TestControl, ZeroInitConditions)
{
    fdcl::state_t *state;
    fdcl::command_t *command;
    fdcl::param *config_file;

    state = new fdcl::state_t();
    command = new fdcl::command_t();

    config_file = new fdcl::param();
    config_file->open("../tests/test.cfg");

    double f, f_out;
    Vector3 M, M_out;

    fdcl::control controller(state, command, config_file);

    state->x.setZero();
    state->v.setZero();
    state->a.setZero();
    state->R.setIdentity();
    state->W.setZero();

    command->xd.setZero();
    command->xd_dot.setZero();
    command->xd_2dot.setZero();
    command->xd_3dot.setZero();
    command->xd_4dot.setZero();
    command->b1d << 1.0, 0.0, 0.0;
    command->b1d_dot.setZero();
    command->b1d_ddot.setZero();

    f = 19.1295;
    M << 0.0, 0.0, 0.0;

    controller.position_control();
    controller.output_fM(f_out, M_out);

    ASSERT_FLOAT_EQ(f, f_out);
    ASSERT_TRUE(M.isApprox(M_out));

    config_file->close();

    delete config_file;
    delete state;
    delete command;
}


TEST(TestControl, NonZeroInitConditions)
{
    fdcl::state_t *state;
    fdcl::command_t *command;
    fdcl::param *config_file;

    state = new fdcl::state_t();
    command = new fdcl::command_t();

    config_file = new fdcl::param();
    config_file->open("../tests/test.cfg");

    double f, f_out;
    Vector3 M, M_out;

    fdcl::control controller(state, command, config_file);

    state->x << 0.0, 1.0, 2.2;
    state->v << 0.0, 1.0, 2.2;
    state->a.setZero();
    state->R << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
    state->W.setZero();

    command->xd << -1.0, 0.0, 0.0;
    command->xd_dot << 0.0, -2.0, 0.0;
    command->xd_2dot << 0.0, 0.0, -5.0;
    command->xd_3dot << 0.0, 1.2, 0.0;
    command->xd_4dot << 2.1, 0.0, 0.0;
    command->b1d << 0.707106781186548, 0.707106781186548, 0.0;
    command->b1d_dot << 0.0, 0.707106781186548, 0.707106781186548;
    command->b1d_ddot << 0.707106781186548, 0.0, 0.707106781186548;

    f = 68.4795;
    M << 0.569002526179056, 0.131284659695342, -0.597667987195412;

    controller.position_control();
    controller.output_fM(f_out, M_out);
    
    // std::cout << M_out << std::endl;

    ASSERT_FLOAT_EQ(f, f_out);
    ASSERT_TRUE(M.isApprox(M_out));

    config_file->close();

    delete config_file;
    delete state;
    delete command;
}