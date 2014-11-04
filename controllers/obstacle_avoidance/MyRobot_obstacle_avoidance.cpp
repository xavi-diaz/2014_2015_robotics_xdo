/**
 * @file    main_template.cpp
 * @brief   A template for webots projects.
 *
 * @author  Name Surname <nick@alumnos.uc3m.es>
 * @date    2014-07
 */

#include "MyRobot_obstacle_avoidance.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 30;

    _left_speed = 0;
    _right_speed = 0;

    _mode = FORWARD;

    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _distance_sensor[0] = getDistanceSensor("ds1");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds2");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds14");
    _distance_sensor[2]->enable(_time_step);
    _distance_sensor[3] = getDistanceSensor("ds13");
    _distance_sensor[3]->enable(_time_step);
    _distance_sensor[4] = getDistanceSensor("ds3");
    _distance_sensor[4]->enable(_time_step);
    _distance_sensor[5] = getDistanceSensor("ds4");
    _distance_sensor[5]->enable(_time_step);
    _distance_sensor[6] = getDistanceSensor("ds12");
    _distance_sensor[6]->enable(_time_step);
    _distance_sensor[7] = getDistanceSensor("ds11");
    _distance_sensor[7]->enable(_time_step);
    _distance_sensor[8] = getDistanceSensor("ds5");
    _distance_sensor[8]->enable(_time_step);
    _distance_sensor[9] = getDistanceSensor("ds6");
    _distance_sensor[9]->enable(_time_step);
    _distance_sensor[10] = getDistanceSensor("ds10");
    _distance_sensor[10]->enable(_time_step);
    _distance_sensor[11] = getDistanceSensor("ds9");
    _distance_sensor[11]->enable(_time_step);
    _distance_sensor[12] = getDistanceSensor("ds0");
    _distance_sensor[12]->enable(_time_step);
    _distance_sensor[13] = getDistanceSensor("ds15");
    _distance_sensor[13]->enable(_time_step);
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
    double s0_val =0.0, s1_val = 0.0, s2_val = 0.0, s3_val = 0.0, s4_val = 0.0, s5_val = 0.0, s6_val = 0.0,
           s9_val = 0.0, s10_val = 0.0, s11_val = 0.0, s12_val = 0.0, s13_val = 0.0, s14_val = 0.0, s15_val = 0.0;
    double compass_angle;
    bool limit_speed = true;


    while (step(_time_step) != -1) {
        // Read the sensors
        s0_val= _distance_sensor[12]->getValue();
        s1_val = _distance_sensor[0]->getValue();
        s2_val = _distance_sensor[1]->getValue();
        s3_val= _distance_sensor[4]->getValue();
        s4_val= _distance_sensor[5]->getValue();
        s5_val= _distance_sensor[8]->getValue();
        s6_val= _distance_sensor[9]->getValue();
        s9_val= _distance_sensor[11]->getValue();
        s10_val= _distance_sensor[10]->getValue();
        s11_val= _distance_sensor[7]->getValue();
        s12_val= _distance_sensor[6]->getValue();
        s13_val= _distance_sensor[3]->getValue();
        s14_val = _distance_sensor[2]->getValue();
        s15_val = _distance_sensor[13]->getValue();

        
        const double *compass_val = _my_compass->getValues();
        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;

        // Control logic of the robot
        if (_mode == FORWARD) {



            // When sufficiently close to a wall in front of robot,
            // switch mode to wall following
            if (((s1_val > DISTANCE_LIMIT || s0_val > DISTANCE_LIMIT) && (s14_val > DISTANCE_LIMIT || s15_val > DISTANCE_LIMIT)) || (s1_val > DISTANCE_LIMIT || s0_val > DISTANCE_LIMIT)) {

                if (s12_val!=0 && s11_val!=0 && s3_val==0 && s4_val==0){
                    _mode = LEFT_WALL_FOLLOWER;
                  cout << "Mode " << LEFT_WALL_FOLLOWER << ": Wall following mode activated" << endl;
                  cout << "Backing up and turning left.(F1)" << endl;
                }
                else{

                  _mode = RIGHT_WALL_FOLLOWER;
                  cout << "Mode " << RIGHT_WALL_FOLLOWER << ": Wall following mode activated" << endl;
                  cout << "Backing up and turning right.(F2)" << endl;
                }
            }

            if ((s14_val  > DISTANCE_LIMIT || s15_val > DISTANCE_LIMIT)&& s1_val==0 && s13_val!=0 && s3_val==0 && s4_val==0){
                _mode = LEFT_WALL_FOLLOWER;
                cout << "Mode " << LEFT_WALL_FOLLOWER << ": Wall following mode activated" << endl;
                cout << "Backing up and turning left.(F3)" << endl;

            }


            else {

                // When there is no sensor activated

                if ((s1_val== 0)&& (s2_val == 0)&& (s14_val == 0)&& (s13_val==0) && (s0_val==0) && (s15_val==0)){

                    if (compass_angle < (DESIRED_ANGLE - 2)){
                        _mode = TURN_RIGHT;
                        cout << "Turning right.(F4)" << endl;

                    }
                    else {
                        if (compass_angle > (DESIRED_ANGLE + 2)){
                            _mode = TURN_LEFT;
                            cout << "Turning left.(F5)" << endl;
                        }
                        else{
                            _mode = FORWARD;
                            cout << "Moving forward.(F6)" << endl;
                        }
                    }

                }
           }

        }
        else {

            if (_mode == TURN_LEFT) {
            
            


                if (s1_val > DISTANCE_LIMIT || s0_val > DISTANCE_LIMIT || s14_val > DISTANCE_LIMIT || s15_val > DISTANCE_LIMIT){
                
                  if(s14_val > DISTANCE_LIMIT || s15_val > DISTANCE_LIMIT || s1_val > DISTANCE_LIMIT || s0_val > DISTANCE_LIMIT){
                      if ((s13_val!=0 || s12_val!=0 || s11_val!=0) && (s3_val==0 && s4_val==0)){
                        _mode = LEFT_WALL_FOLLOWER;
                        cout << "Mode " << LEFT_WALL_FOLLOWER << ": Wall following mode activated" << endl;
                        cout << "Backing up and turning left.(L1)" << endl;
                      }
                      else{

                          if ((s2_val!=0 || s3_val!=0 || s4_val!=0) && (s12_val==0 && s11_val==0)){

                            _mode = RIGHT_WALL_FOLLOWER;
                            cout << "Mode " << RIGHT_WALL_FOLLOWER << ": Wall following mode activated" << endl;
                            cout << "Backing up and turning right.(L2)" << endl;
                          }

                          else{
                            _mode = RIGHT_WALL_FOLLOWER;
                            cout << "Mode " << RIGHT_WALL_FOLLOWER << ": Wall following mode activated" << endl;
                            cout << "Backing up and turning right.(L3)" << endl;
                          }
                      }
                  }
                  
                  else{
                    limit_speed = true;
                    _mode = TURN_RIGHT;
                    cout << "Turning right.(L4)" << endl;
                  }
                }
                
                if (s2_val > DISTANCE_LIMIT && s14_val==0) {
                     limit_speed = true;
                     _mode = TURN_RIGHT;
                     cout << "Turning right.(L5)" << endl;
                }


                else{

                    if ((s1_val== 0)&& (s2_val == 0)&& (s14_val == 0)&& (s13_val==0) && (s0_val==0) && (s15_val==0)){

                        limit_speed = false;

                        if (compass_angle < (DESIRED_ANGLE - 2)){
                            _mode = TURN_RIGHT;
                            cout << "Turning right.(L6)" << endl;
                        }
                        else {
                            if (compass_angle > (DESIRED_ANGLE + 2)){
                                _mode = TURN_LEFT;
                                cout << "Turning left.(L7)" << endl;
                            }
                            else{
                                _mode = FORWARD;
                                cout << "Moving forward.(L8)" << endl;
                            }
                        }

                    }

                    else{

                        if((s13_val < DISTANCE_LIMIT + 50) && s12_val!=0 && s11_val!=0 && s2_val ==0){
                             limit_speed = true;
                             _mode = TURN_RIGHT;
                             cout << "Turning right.(L9)" << endl;
                        }

                    }



                }


            }

            else{

                if(_mode == TURN_RIGHT){


                    if (s1_val > DISTANCE_LIMIT || s0_val > DISTANCE_LIMIT || s14_val > DISTANCE_LIMIT || s15_val > DISTANCE_LIMIT){

                      if(s14_val > DISTANCE_LIMIT || s15_val > DISTANCE_LIMIT || s1_val > DISTANCE_LIMIT || s0_val > DISTANCE_LIMIT){
                          if ((s13_val!=0 || s12_val!=0 || s11_val!=0) && (s3_val==0 && s4_val==0)){
                            _mode = LEFT_WALL_FOLLOWER;
                            cout << "Mode " << LEFT_WALL_FOLLOWER << ": Wall following mode activated" << endl;
                            cout << "Backing up and turning left.(R1)" << endl;
                          }
                          else{

                              if ((s2_val!=0 || s3_val!=0 || s4_val!=0) && (s12_val==0 && s11_val==0)){

                                _mode = RIGHT_WALL_FOLLOWER;
                                cout << "Mode " << RIGHT_WALL_FOLLOWER << ": Wall following mode activated" << endl;
                                cout << "Backing up and turning right.(R2)" << endl;
                              }

                              else{
                                 _mode = RIGHT_WALL_FOLLOWER;
                                 cout << "Mode " << RIGHT_WALL_FOLLOWER << ": Wall following mode activated" << endl;
                                 cout << "Backing up and turning right.(R3)" << endl;

                              }
                          }
                      }

                      else{
                        limit_speed = true;
                        _mode = TURN_LEFT;
                        cout << "Turning left.(R4)" << endl;
                      }
                    }







                    if(s13_val > DISTANCE_LIMIT && s1_val==0){
                        limit_speed = true;
                        _mode = TURN_LEFT;
                        cout << "Turning left.(R5)" << endl;
                    }



                    else{

                        if ((s1_val== 0)&& (s2_val == 0)&& (s14_val == 0)&& (s13_val==0)&& (s0_val==0) && (s15_val==0)){

                            limit_speed = false;

                            if (compass_angle < (DESIRED_ANGLE - 2)){
                                _mode = TURN_RIGHT;
                                cout << "Turning right.(R6)" << endl;
                            }
                            else {
                                if (compass_angle > (DESIRED_ANGLE + 2)){
                                    _mode = TURN_LEFT;
                                    cout << "Turning left.(R7)" << endl;
                                }
                                else{
                                    _mode = FORWARD;
                                    cout << "Moving forward.(R8)" << endl;
                                }
                            }

                        }

                        else{

                            if((s2_val < DISTANCE_LIMIT + 50)&& s3_val!=0 && s4_val!=0 && s13_val ==0){
                                 limit_speed = true;
                                 _mode = TURN_LEFT;
                                 cout << "Turning left.(R9)" << endl;
                            }
                        }


                    }


                }


                else {

                    if (_mode == RIGHT_WALL_FOLLOWER){

                        if(s0_val > DISTANCE_LIMIT || s15_val > DISTANCE_LIMIT){
                                _mode = RIGHT_WALL_FOLLOWER;
                                cout << "Backing up and turning right.(RWF1)" << endl;
                        }

                        else{

                            if ((s1_val > DISTANCE_LIMIT ) || (s14_val > DISTANCE_LIMIT) ) {
                                _mode = RIGHT_WALL_FOLLOWER;
                                cout << "Backing up and turning right.(RWF2)" << endl;
                            }



                            else{

                                if (s2_val!=0 && s4_val!=0){

                                    limit_speed = true;

                                    if (s2_val > DISTANCE_LIMIT) {
                                        _mode = TURN_RIGHT;
                                        cout << "Turning right.(RWF3)" << endl;
                                    }
                                    else {
                                        if (s2_val < DISTANCE_LIMIT + 50) {
                                            _mode = TURN_LEFT;
                                            cout << "Turning left.(RWF4)" << endl;
                                        }

                                    }

                                }

                                else {
                                    _mode = FORWARD;
                                    cout << "Moving forward.(RWF5)" << endl;
                                }


                            }
                       }


                    }

                    else{

                        if (_mode == LEFT_WALL_FOLLOWER){


                            if(s0_val > DISTANCE_LIMIT || s15_val > DISTANCE_LIMIT){
                                 _mode = LEFT_WALL_FOLLOWER;
                                  cout << "Backing up and turning left.(LWF1)" << endl;
                            }

                            else{

                                if ((s1_val > DISTANCE_LIMIT) || (s14_val > DISTANCE_LIMIT)) {
                                    _mode = LEFT_WALL_FOLLOWER;
                                    cout << "Backing up and turning left.(LWF2)" << endl;
                                }




                                else{


                                    if(s13_val!=0 && s11_val!=0){
                                        limit_speed = true;

                                        if(s13_val > DISTANCE_LIMIT){
                                            _mode = TURN_LEFT;
                                            cout << "Turning left.(LWF3)" << endl;

                                        }
                                        else {
                                            if (s13_val < DISTANCE_LIMIT + 50) {
                                                _mode = TURN_RIGHT;
                                                cout << "Turning right.(LWF4)" << endl;
                                            }

                                        }

                                    }

                                    else {
                                        _mode = FORWARD;
                                        cout << "Moving forward.(LWF5)" << endl;
                                    }


                                }

                             }


                        }
                    }


                }


            }

        }





        // Send actuators commands according to the mode
        switch (_mode){
           

            case STOP:
                _left_speed = 0;
                _right_speed = 0;
                break;
            case FORWARD:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
                break;
            case TURN_LEFT:
                if(limit_speed){
                    _left_speed = MAX_SPEED / 2.5;
                    _right_speed = MAX_SPEED/2;
                }
                else{
                    _left_speed = MAX_SPEED - 15;
                    _right_speed = MAX_SPEED;
                }
                break;
            case TURN_RIGHT:
                if(limit_speed){
                    _left_speed = MAX_SPEED/2;
                    _right_speed = MAX_SPEED / 2.5;
                }
                else{
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED - 15;
                }
                break;
            case RIGHT_WALL_FOLLOWER:
                _left_speed = -MAX_SPEED / 40.0;
                _right_speed = -MAX_SPEED / 6.0;
                break;
            case LEFT_WALL_FOLLOWER:
                _left_speed = -MAX_SPEED / 6.0;
                _right_speed = -MAX_SPEED / 40.0;
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
