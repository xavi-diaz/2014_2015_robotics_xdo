/**
 * @file    MyRobot.cpp
 * @brief   Controller for detecting a yellow line near robot using the spherical camera.
 *
 * @author  Xavier Diaz Ortiz <100293639@alumnos.uc3m.es>
 * @date    2014-11
 */

#include "MyRobot.h"

/**
 * @brief Main program.
 */
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
