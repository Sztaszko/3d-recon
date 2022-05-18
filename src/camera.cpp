#include "camera.h"
#include <iostream>
#include <cmath>

const double PI = 3.14159265;


cameraAPI::Camera::Camera() :
    cv::VideoCapture()
  , _posX(0)
  , _posY(0)
  , _posZ(0)
  , _roll(0)
  , _pitch(0)
  , _yaw(0)
{

}

void cameraAPI::Camera::init(std::vector<double> position)
{
    try {
        _posX = position.at(0);
        _posY = position.at(1);
        _posZ = position.at(2);
        _roll = position.at(3);
        _pitch = position.at(4);
        _yaw = position.at(5);
    } catch (std::out_of_range &err) {
        std::cerr << "Init vector error: " << err.what() << "\n";
        std::cout << "Camera position assumed at: " << _posX, _posY, _posZ, _roll, _pitch, _yaw;
    }
}


cameraAPI::HandCamera::HandCamera() :
    Camera()
{
}

std::vector<double> cameraAPI::HandCamera::move(double x, double y, double z)
{
    int input;
    std::cout << "Camera move requested. Please move camera by " << _posX - x << " m";
    std::cout << "Input actual translated value: ";
    std::cin >> input;

    _posX += input;

    // TODO rotation - get from user or calculate from images

    return { _posX, _posY, _posZ, _roll, _pitch, _yaw };
}

