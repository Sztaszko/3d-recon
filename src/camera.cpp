#include "camera.h"
#include <iostream>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <condition_variable>

const double PI = 3.14159265;


cameraAPI::CameraPosition::CameraPosition() :
    _posX(0)
  , _posY(0)
  , _posZ(0)
  , _roll(0)
  , _pitch(0)
  , _yaw(0)
{

}

void cameraAPI::CameraPosition::init(std::vector<double> position)
{
    _camera_positions.clear();

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

    _camera_positions.push_back(get_current_camera_position());
}

cv::Mat cameraAPI::CameraPosition::get_current_camera_position()
{
    cv::Mat t({_posX, _posY, _posZ});
    cv::Mat rotVec({_roll, _pitch, _yaw});
    cv::Mat R;
    cv::Rodrigues(rotVec, R);
    cv::transpose(R, R); // R is rotation so we can transpose instead of inverse

    return -(R)*t;
}

cv::Matx44f cameraAPI::CameraPosition::get_current_extrinsic_matrix()
{
    cv::Mat t({_posX, _posY, _posZ});
    cv::Mat rotVec({_roll, _pitch, _yaw});
    cv::Mat R;
    cv::Rodrigues(rotVec, R);

    cv::Matx44f extrinsic = cv::Matx44f(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
                                         R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
                                         R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2),
                                         0,                 0,                 0,                 1);

    return extrinsic;
}

cv::Mat cameraAPI::CameraPosition::get_camera_position(int index)
{
    cv::Mat posMat;
    try {
    posMat = _camera_positions.at(index);

    } catch (const std::out_of_range &e) {
        std::cout << "Index out of range return empty position matrix: " << e.what() << "\n";
        posMat = cv::Mat();
    }

    return posMat;
}


cameraAPI::HandCameraPosition::HandCameraPosition() :
    CameraPosition()
{
}

std::vector<double> cameraAPI::HandCameraPosition::move(double x, double y, double z)
{
    int input;
    std::cout << "Camera move requested. Please move camera by " << x << " m\n";
    std::cout << "Input actual translated value: ";
    std::cin >> input;
    std::cout << std::endl;

    _posX += input;

    // TODO rotation - get from user or calculate from images

    _camera_positions.push_back(get_current_camera_position());

    return { _posX, _posY, _posZ, _roll, _pitch, _yaw };
}

std::vector<double> cameraAPI::HandCameraPosition::move(double x)
{
    return move(x, 0, 0);
}



/* ============== CameraThread ============= */

cv::Mat cameraAPI::CameraThread::read()
{

    cv::Mat frame = _frames_queue.dequeue();

    return frame;
}

void cameraAPI::CameraThread::init(int device_id, int api_id)
{
    _device_id = device_id;
    _api_id = api_id;
}

void cameraAPI::CameraThread::start()
{
    if (_running) {
        std::cout << "Thread is already running!\n";
        return ;
    }

    _running = true;
    _acquisition_thread.reset(new std::thread(&CameraThread::_read, this));

}

void cameraAPI::CameraThread::stop()
{
    if (!_running) {
        std::cout << "Thread is already stopped.\n";
        return;
    }
    _running = false;
    _acquisition_thread->join();
}


void cameraAPI::CameraThread::_read()
{
    if (!_camera.open(_device_id, _api_id)) {
        std::cerr << "ERROR! Unable to open camera. Not starting acquisition thread.\n";
        return ;
    }

    while (_running) {
        cv::Mat frame;
        if (!_camera.read(frame)) {
            std::cerr << "Camera read failed. Ending acquisition";
            break;
        }

        if (!_frames_queue.empty()) {
            _frames_queue.dequeue(); // discarding unproccessed frame
        }
        _frames_queue.enqueue(frame);

    }

    _camera.release();
}

