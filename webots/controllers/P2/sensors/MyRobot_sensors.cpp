/**
 * @file    MyRobot.cpp
 * @brief   A simple example for maintaining a straight line with the compass.
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date    2014-07
 */

#include "MyRobot_sensors.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds2");
    _distance_sensor[2]->enable(_time_step);
    _distance_sensor[3] = getDistanceSensor("ds15");
    _distance_sensor[3]->enable(_time_step);
    _distance_sensor[4] = getDistanceSensor("ds14");
    _distance_sensor[4]->enable(_time_step);
    _distance_sensor[5] = getDistanceSensor("ds13");
    _distance_sensor[5]->enable(_time_step);
}


//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _my_compass->disable();

    for (int i=0; i<6; i++) {
        _distance_sensor[i]->disable();
    }
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double compass_angle;
    double sensor_value;


    while (step(_time_step) != -1) {
        // Read the sensors
        const double *compass_val = _my_compass->getValues();


        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);


        // Print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl <<endl;

        for(int i =0; i<6;i ++){
            sensor_value = _distance_sensor[i]->getValue();
            cout<<"Frontal sensor ["<< i+1 << "] vaue: "<<sensor_value <<endl;
        }

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
            }
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

//////////////////////////////////////////////
