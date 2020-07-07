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

    fdcl::control control(state, command, config_file);

    std::cout << "Controller initialized" << std::endl;

    config_file->close();

    delete config_file;
    delete state;
    delete command;

    return 0;
}
