/**
 * @file    MyRobot_obstacle_avoidance.h
 * @brief   Controller to make the robot reach the other side of the world avoiding obstacles.
 *
 * @author  Xavier Diaz <100293639@alumnos.uc3m.es>
 * @date    2014-11
 */


#include <string>
#include<sstream>
#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 16
#define DISTANCE_LIMIT      100
#define DESIRED_ANGLE  45.0
#define OPPOSITE_DESIRED_ANGLE  -135.0
#define MAX_SPEED           100

class MyRobot : public DifferentialWheels {
    private:
        int _time_step;	
        Compass * _my_compass;
        DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];
        double _left_speed, _right_speed;

        enum Mode {
            STOP,
            FORWARD,
            TURN_LEFT,
            TURN_RIGHT,
            RIGHT_WALL_FOLLOWER,
            LEFT_WALL_FOLLOWER,
            GO_BACK,
            TURN_AROUND	   
        };

        Mode _mode;

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
         * @brief Function that describes the robot's behaviour in its environment.
         */
        void run();

        /**
         * @brief Converts compass information to degrees.
         * @param in_vector Compass data.
         * @return Angle in degrees.
         */
        double convert_bearing_to_degrees(const double* in_vector);

        /**
         * @brief Reads distance sensors values.
         * @param s Array that will store the sensors values.
         */
        void read_sensors(double s[]);

        /**
         * @brief Represents the default motion of the robot to its target when there is no obstacle along its trajectory.
         * @param s Array that stores the sensors values.
         * @param compass_angle Angle in degrees that determines the robot's orientation.
         * @param limit_speed Variable that if true reduces the speed when robot is in FORWARD, TURN_LEFT or TURN_RIGHT mode.
         * @param just_turned_around Indicates the robot has just rotated around itself.
         * @param avoid_turning_around_again Prevents robot from doing a pointless rotation one more time and lets it follow its current path.
         */
        void move_towards_desired_direction(double s[], double compass_angle, bool& limit_speed, bool& just_turned_around, bool& avoid_turning_around_again);


        /**
         * @brief Explains in which situations the robot goes back when it is likely to get stuck.
         * @param s Array that stores the sensors values.
         */
        void conditions_for_going_back(double s[]);


        /**
         * @brief Specifies how the robot decides to enter either into LEFT-or-RIGHT_WALL_FOLLOWER mode.
         * @param s Array that stores the sensors values.
         */
        void switch_to_wall_follower(double s[]);

};
