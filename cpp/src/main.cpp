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

#include <iostream>
#include <vector>

fdcl::state_t *state;
fdcl::command_t *command;
fdcl::param *config_file;

int main(int argc, char **argv)
{
    state = new fdcl::state_t();
    command = new fdcl::command_t();

    config_file = new fdcl::param();
    config_file->open("../uav.cfg");

    fdcl::control controller(state, command, config_file);
    std::cout << "Controller initialized" << std::endl;

    double f_out;
    Vector3 M_out;

    for (int i = 0; i < 10; i++)
    {
        state->x << 0.0, 0.0, i;
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

        controller.position_control();
        controller.output_fM(f_out, M_out);

        std::cout << "i = " << i << ":\tf = " << f_out << std::endl;
    }

    config_file->close();

    delete config_file;
    delete state;
    delete command;

    return 0;
}
