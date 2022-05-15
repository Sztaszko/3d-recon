#include "camera.h"
#include <iostream>
#include <cmath>

const double PI = 3.14159265;

cameraAPI::HandCamera::HandCamera() :
    Camera()
{

}

std::vector<double> cameraAPI::HandCamera::move(double x, double y, double z)
{
    // assume radius is 1 m and object is in 0,0,0 position
    // we move only in XZ plane, not changing Y


    std::cout << "Camera move requested. Please ";
}

std::vector<double> cameraAPI::HandCamera::move(int alpha)
{
    //assume moving only in XZ plane by the circle with radius of 1 m

    int input;

    std::cout << "Camera move requested. Please rotate disc by " << alpha << " degrees.";
    std::cout << "Type in the real roteted value: ";
    std::cin >> input;


    double angle_rad = (PI*input)/180;

    _posX += sin(angle_rad);
    _posZ += cos(angle_rad);

    return { _posX, _posY, _posZ };
}

cameraAPI::Camera::Camera() :
    _posX(0)
  , _posY(0)
  , _posZ(0)
{

}
