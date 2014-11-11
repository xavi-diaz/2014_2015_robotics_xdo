/**
 * @file    MyRobot_obstacle_avoidance.cpp
 * @brief   Controller to make the robot reach the other side of the world avoiding obstacles.
 *
 * @author  Xavier Diaz <100293639@alumnos.uc3m.es>
 * @date    2014-11
 */

#include "MyRobot_obstacle_avoidance.h"


//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _mode = FORWARD;

    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);


    for(int i=0; i<NUM_DISTANCE_SENSOR; i++){
        string str;
        stringstream ss;
        ss << i;
        str = "ds";
        str += ss.str();
        _distance_sensor[i] = getDistanceSensor(str);
        _distance_sensor[i]->enable(_time_step);
    }

}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{

    _my_compass->disable();

    for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
        _distance_sensor[i]->disable();

}

//////////////////////////////////////////////

void MyRobot::run()
{

    double compass_angle;
    double s[NUM_DISTANCE_SENSOR];
    bool limit_speed = true;
    bool just_turned_around = false;
    bool avoid_turning_around_again = false;


    while (step(_time_step) != -1) {


        // Read distance sensors values
        read_sensors(s);
        
        const double *compass_val = _my_compass->getValues();
        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;
        cout << "limit speed = " << limit_speed << endl;
        cout << "s0 = " << s[0] << " and s15 = " << s[15] << endl;
        cout << "s1 = " << s[1] << " and s14 = " << s[14] << endl;
        cout << "s2 = " << s[2] << " and s13 = " << s[13] << endl;
        cout << "s3 = " << s[3] << " and s12 = " << s[12] << endl;
        cout << "s4 = " << s[4] << " and s11 = " << s[11] << endl;
        cout << "s5 = " << s[5] << " and s10 = " << s[10] << endl;
        cout << "s6 = " << s[6] << " and s9 = " << s[9] << endl;





        // Control logic of the robot
        if (_mode == FORWARD) {


            // if robot "feels" that it is going to get stuck, then...
            conditions_for_going_back(s);



            // When sufficiently close to a wall in front of robot,
            // switch mode to wall following
            if ((s[1] > DISTANCE_LIMIT || s[0] > DISTANCE_LIMIT || s[14] > DISTANCE_LIMIT || s[15] > DISTANCE_LIMIT) && (s[6]<(DISTANCE_LIMIT*10)+24) && (s[9]<(DISTANCE_LIMIT*10)+24)){

                switch_to_wall_follower(s);
            }

            else {

                move_towards_desired_direction(s,compass_angle, limit_speed, just_turned_around, avoid_turning_around_again);
            }
        }
        else {

            if (_mode == TURN_LEFT) {


                conditions_for_going_back(s);

                if ((s[1] > DISTANCE_LIMIT || s[0] > DISTANCE_LIMIT || s[14] > DISTANCE_LIMIT || s[15] > DISTANCE_LIMIT) && (s[6]<(DISTANCE_LIMIT*10)+24) && (s[9]<(DISTANCE_LIMIT*10)+24)){

                    switch_to_wall_follower(s);
                }


                //When robot realizes it is going in the opposite direction
                //and if it has NOT just turned around
                if((s[3]!=0 || s[4]!=0 || s[12]!=0 || s[11]!=0) && s[7]==0 && s[8]==0 && !avoid_turning_around_again){
                    if((OPPOSITE_DESIRED_ANGLE-2) <= compass_angle && compass_angle <= (OPPOSITE_DESIRED_ANGLE+2)){
                        if(just_turned_around)
                            avoid_turning_around_again = true;
                        _mode = TURN_AROUND;
                        cout << "Turning around.(L1)" << endl;
                    }
                }

                if (s[2] > DISTANCE_LIMIT && s[14]==0) {
                    limit_speed = true;
                    _mode = TURN_RIGHT;
                    cout << "Turning right.(L2)" << endl;
                }

                else{
                    if((s[13] < DISTANCE_LIMIT + 50) && s[12]!=0 && s[11]!=0 && s[2]==0){
                        limit_speed = true;
                        _mode = TURN_RIGHT;
                        cout << "Turning right.(L3)" << endl;
                    }

                    else{
                        move_towards_desired_direction(s,compass_angle, limit_speed, just_turned_around, avoid_turning_around_again);
                    }
                }
            }
            else{

                if(_mode == TURN_RIGHT){


                    conditions_for_going_back(s);

                    if ((s[1] > DISTANCE_LIMIT || s[0] > DISTANCE_LIMIT || s[14] > DISTANCE_LIMIT || s[15] > DISTANCE_LIMIT) && (s[6]<(DISTANCE_LIMIT*10)+24) && (s[9]<(DISTANCE_LIMIT*10)+24)){

                        switch_to_wall_follower(s);
                    }

                    if((s[3]!=0 || s[4]!=0 || s[12]!=0 || s[11]!=0) && s[7]==0 && s[8]==0 && !avoid_turning_around_again){
                        if((OPPOSITE_DESIRED_ANGLE-2) <= compass_angle && compass_angle <= (OPPOSITE_DESIRED_ANGLE+2)){
                            if (just_turned_around)
                                avoid_turning_around_again = true;
                            _mode = TURN_AROUND;
                            cout << "Turning around.(R1)" << endl;
                        }
                    }


                    if(s[13] > DISTANCE_LIMIT && s[1]==0){
                        limit_speed = true;
                        _mode = TURN_LEFT;
                        cout << "Turning left.(R2)" << endl;
                    }



                    else{
                        if((s[2] < DISTANCE_LIMIT + 50)&& s[3]!=0 && s[4]!=0 && s[13]==0){
                            limit_speed = true;
                            _mode = TURN_LEFT;
                            cout << "Turning left.(R3)" << endl;
                        }

                        else{
                            move_towards_desired_direction(s,compass_angle, limit_speed, just_turned_around, avoid_turning_around_again);
                        }
                    }
                }

                else {

                    if (_mode == RIGHT_WALL_FOLLOWER){


                        conditions_for_going_back(s);

                        if((s[0] > DISTANCE_LIMIT || s[15] > DISTANCE_LIMIT || s[1] > DISTANCE_LIMIT || s[14] > DISTANCE_LIMIT) && s[6]<(DISTANCE_LIMIT*10) && s[9]<(DISTANCE_LIMIT*10)){
                            _mode = RIGHT_WALL_FOLLOWER;
                            cout << "Backing up and turning right.(RWF1)" << endl;
                        }

                        else{

                            limit_speed = true;

                            if (s[2]!=0 && s[4]!=0){

                                if (s[2] > DISTANCE_LIMIT) {
                                    _mode = TURN_RIGHT;
                                    cout << "Turning right.(RWF2)" << endl;
                                }
                                else {
                                    if (s[2] < DISTANCE_LIMIT + 50) {
                                        _mode = TURN_LEFT;
                                        cout << "Turning left.(RWF3)" << endl;
                                    }
                                }
                            }
                            else {
                                _mode = FORWARD;
                                cout << "Moving forward.(RWF4)" << endl;
                            }
                        }
                    }
                    else{

                        if (_mode == LEFT_WALL_FOLLOWER){

                            conditions_for_going_back(s);

                            if((s[0] > DISTANCE_LIMIT || s[15] > DISTANCE_LIMIT || s[1] > DISTANCE_LIMIT || s[14] > DISTANCE_LIMIT) && s[6]<(DISTANCE_LIMIT*10) && s[9]<(DISTANCE_LIMIT*10)){
                                _mode = LEFT_WALL_FOLLOWER;
                                cout << "Backing up and turning left.(LWF1)" << endl;
                            }
                            else{

                                limit_speed = true;

                                if(s[13]!=0 && s[11]!=0){

                                    if(s[13] > DISTANCE_LIMIT){
                                        _mode = TURN_LEFT;
                                        cout << "Turning left.(LWF2)" << endl;
                                    }
                                    else {
                                        if (s[13] < DISTANCE_LIMIT + 50) {
                                            _mode = TURN_RIGHT;
                                            cout << "Turning right.(LWF3)" << endl;
                                        }
                                    }
                                }

                                else {
                                    _mode = FORWARD;
                                    cout << "Moving forward.(LWF4)" << endl;
                                }
                            }
                        }
                        else{

                            if(_mode == GO_BACK){
                                if(s[3]==0 && !s[12]){
                                    _mode = TURN_RIGHT;
                                    cout << "Turning right.(GB1)" << endl;
                                }
                                else{
                                    if(s[12]==0 && !s[3]){
                                        _mode = TURN_LEFT;
                                        cout << "Turning left.(GB2)" << endl;
                                    }
                                }
                            }

                            else{

                                if(_mode == TURN_AROUND){

                                    if((DESIRED_ANGLE-2 <= compass_angle) && (compass_angle<= DESIRED_ANGLE+2)){
                                        limit_speed = true;
                                        _mode = FORWARD;
                                        cout << "Moving forward.(TA)" << endl;
                                        just_turned_around = true;
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
            if(limit_speed){
                _left_speed = MAX_SPEED/2;
                _right_speed = MAX_SPEED/2;
            }
            else{
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
            }
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
        case GO_BACK:
            _left_speed = -MAX_SPEED;
            _right_speed = -MAX_SPEED;
            break;
        case TURN_AROUND:
            _left_speed = -MAX_SPEED*0.1;
            _right_speed = MAX_SPEED*0.1;
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


//////////////////////////////////////////////
void MyRobot::read_sensors(double s[]){

    for(int i=0; i<NUM_DISTANCE_SENSOR; i++)
        s[i]= _distance_sensor[i]->getValue();

}


//////////////////////////////////////////////
void MyRobot::move_towards_desired_direction(double s[], double compass_angle, bool& limit_speed, bool& just_turned_around, bool& avoid_turning_around_again){

    // When there is no frontal and lateral sensor activated

    if ((s[1]==0) && (s[2]==0) && (s[14]==0) && (s[13]==0) && (s[0]==0) && (s[15]==0) && (s[3]==0) && (s[4]==0) && (s[12]==0) && (s[11]==0)){

        just_turned_around = false;
        avoid_turning_around_again = false;
        limit_speed = false;

        if (compass_angle < (DESIRED_ANGLE - 2)){
            _mode = TURN_RIGHT;
            cout << "Turning right." << endl;

        }
        else {
            if (compass_angle > (DESIRED_ANGLE + 2)){
                _mode = TURN_LEFT;
                cout << "Turning left." << endl;
            }
            else{
                _mode = FORWARD;
                cout << "Moving forward." << endl;
            }
        }

    }

}


//////////////////////////////////////////////
void MyRobot::conditions_for_going_back(double s[]){

    if(s[0]==0 && s[1]==0  && s[5]==0 && s[6]==0 && (s[3]>=(DISTANCE_LIMIT*10)+24) && s[4]<DISTANCE_LIMIT){
        cout << "Going Back..." << endl;
        _mode = GO_BACK;
    }

    if(s[15]==0 && s[14]==0  && s[10]==0 && s[9]==0 && (s[12]>=(DISTANCE_LIMIT*10)+24) && s[11]<DISTANCE_LIMIT){
        cout << "Going Back..." << endl;
        _mode = GO_BACK;
    }

}



//////////////////////////////////////////////
void MyRobot::switch_to_wall_follower(double s[]){

    if (((s[13]!=0 || s[12]!=0 || s[9]!=0) && ( s[3]==0 && s[4]==0 && s[5]==0)) || (s[1]==0 && s[2]==0 && s[3]==0 && s[14]!=0) || (s[2]==0 && s[1]==0 && s[0]==0 && s[15]!=0 && s[14]==0 && s[13]==0 )){
        _mode = LEFT_WALL_FOLLOWER;
        cout << "Mode " << LEFT_WALL_FOLLOWER << ": Wall following mode activated" << endl;
        cout << "Backing up and turning left." << endl;
    }
    else{

        if (((s[2]!=0 || s[3]!=0 || s[6]!=0) && ( s[12]==0 && s[11]==0 && s[10]==0)) || (s[13]==0 && s[14]==0 && s[12]==0  && s[1]!=0) || (s[2]==0 && s[1]==0 && s[0]!=0 && s[15]==0 && s[14]==0 && s[13]==0 )){

            _mode = RIGHT_WALL_FOLLOWER;
            cout << "Mode " << RIGHT_WALL_FOLLOWER << ": Wall following mode activated" << endl;
            cout << "Backing up and turning right." << endl;
        }
        else{
            _mode = RIGHT_WALL_FOLLOWER;
            cout << "Mode " << RIGHT_WALL_FOLLOWER << ": Wall following mode activated" << endl;
            cout << "Backing up and turning right." << endl;
        }
    }

}
