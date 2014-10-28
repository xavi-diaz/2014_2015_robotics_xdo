/**
 * @file    main_template.cpp
 * @brief   A template for webots projects.
 *
 * @author  Name Surname <nick@alumnos.uc3m.es>
 * @date    2014-07
 */

#include "MyRobot_wall_follower.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _mode = START;

    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _distance_sensor[0] = getDistanceSensor("ds1");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds2");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds14");
    _distance_sensor[2]->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{

    _my_compass->disable();

    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double s1_val = 0.0, s2_val = 0.0, s14_val = 0.0;
    double compass_angle;

    while (step(_time_step) != -1) {
        // Read the sensors
        s1_val = _distance_sensor[0]->getValue();
        s2_val = _distance_sensor[1]->getValue();
        s14_val = _distance_sensor[2]->getValue();
        
        const double *compass_val = _my_compass->getValues();
        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Control logic of the robot
        if (_mode == START) {
            // Move towards DESIRED_ANGLE angle direction

            // When sufficiently close to a wall in front of robot,
            // switch mode to wall following
            if ((s1_val > DISTANCE_LIMIT) || (s14_val > DISTANCE_LIMIT)) {
                _mode = WALL_FOLLOWER;
                cout << "Mode " << WALL_FOLLOWER << ": Wall following mode activated" << endl;
            }
        }
        else {
            // Wall following

            if ((s1_val > DISTANCE_LIMIT) || (s14_val > DISTANCE_LIMIT)) {
                _mode = WALL_FOLLOWER;
                cout << "Backing up and turning right." << endl;
            }
            else {
                if (s2_val > DISTANCE_LIMIT) {
                    _mode = TURN_RIGHT;
                    cout << "Turning right." << endl;
                }
                else {
                    if (s2_val < DISTANCE_LIMIT + 50) {
                        _mode = TURN_LEFT;
                        cout << "Turning left." << endl;
                    }
                    else {
                        _mode = FORWARD;
                        cout << "Moving forward." << endl;
                    }
                }
            }
        }

        // Send actuators commands according to the mode
        switch (_mode){
            case START:
        

                // Print sensor values to console
                cout << "Compass angle (degrees): " << compass_angle << endl;

                // Simple bang-bang control
                if (compass_angle < (DESIRED_ANGLE - 2)) {
                    // Turn right
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED - 15;
                }
                else {
                    if (compass_angle > (DESIRED_ANGLE + 2)) {
                        // Turn left
                        _left_speed = MAX_SPEED - 15;
                        _right_speed = MAX_SPEED;
                    }
                    else {
                        // Move straight forward
                        _left_speed = MAX_SPEED;
                        _right_speed = MAX_SPEED;
                        cout << "Moving forward." << endl;
                    }
                }
                break;

            case STOP:
                _left_speed = 0;
                _right_speed = 0;
                break;
            case FORWARD:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
                break;
            case TURN_LEFT:
                _left_speed = MAX_SPEED / 1.25;
                _right_speed = MAX_SPEED;
                break;
            case TURN_RIGHT:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED / 1.25;
                break;
            case WALL_FOLLOWER:
                _left_speed = -MAX_SPEED / 20.0;
                _right_speed = -MAX_SPEED / 3.0;
                break;
            default:
                break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);


    }
}

//////////////////////////////////////////////
double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}
