/**
 * @file    MyRobot.cpp
 * @brief   Controller for detecting a yellow line near robot using the spherical camera.
 *
 * @author  Xavier Diaz Ortiz <100293639@alumnos.uc3m.es>
 * @date    2014-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    // init default values
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // get distance sensor array and enable each one
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable devices
    _forward_camera->disable();
    _spherical_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    int sum = 0;
    unsigned char green = 0, red = 0, blue = 0;
    double percentage_yellow = 0.0;

    // get size of images for forward camera
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = _forward_camera->getHeight();
    cout << "Size of forward camera image: " << image_width_f << ", " <<  image_height_f << endl;

    // get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();
    cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;

    while (step(_time_step) != -1) {
        sum = 0;

        // get current image from spherical camera
        const unsigned char *image_s = _spherical_camera->getImage();

        // count number of pixels that are yellow
        // (here assumed to have green and red pixel value > 230 and blue pixel value = 0 (out of 255) for all color components)
        for (int x = 0; x < image_width_s; x++) {
            for (int y = 0; y < image_height_s; y++) {
                green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
                red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
                blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);


                // 230 for green and red is a value from which one could state that the range of yellow colors begin
                // (of course making a personal approximmation in the RGB scale)
                if ((green > 230) && (red > 230) && (blue == 0)) {
                    sum = sum + 1;
                }
            }
        }

        percentage_yellow = (sum / (float) (image_width_s * image_height_s)) * 100;
        cout << "Percentage of yellow in spherical camera image: " << percentage_yellow << endl;


        // 0.21% is an empirical threeshold value that considers the robot is sufficiently near a line and was obtained after
        // several simulation tests but how close may actually vary depending on the location of the surrounding objects
        if (percentage_yellow >= 0.21)
            cout << "A yelow line near robot is being detected" << endl;

        // turn around slowly
        _left_speed = 5;
        _right_speed = -5;

        // set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////
