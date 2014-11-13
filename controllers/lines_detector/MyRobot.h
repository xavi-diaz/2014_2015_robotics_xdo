#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   Controller for detecting a yellow line near robot using the spherical camera.
 *
 * @author  Xavier Diaz Ortiz <100293639@alumnos.uc3m.es>
 * @date    2014-11
 */

#include <iostream>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 2
#define DISTANCE_LIMIT      100
#define MAX_SPEED           100

class MyRobot : public DifferentialWheels {
    public:
        // You may need to define your private methods or variables, like
        //  Constructors, helper functions, etc.

        /**
         * @brief Empty constructor of the class.
         */
        MyRobot();

        /**
         * @brief Destructor of the class.
         */
        ~MyRobot();

        /**
         * @brief Function with the logic of the controller.
         * @param
         * @return
         */
        void run();
        
    private:
        int _time_step;
        
        // velocities
        double _left_speed, _right_speed;

        // sensors
        Camera *_forward_camera;
        Camera *_spherical_camera;
};

#endif

