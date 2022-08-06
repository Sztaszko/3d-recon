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
    clear();

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
    _camera_extrinsics.push_back(get_current_extrinsic_matrix());
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

cv::Matx44f cameraAPI::CameraPosition::get_camera_extrinsic(int index)
{
    cv::Matx44f extrMat;
    try {
    extrMat = _camera_extrinsics.at(index);

    } catch (const std::out_of_range &e) {
        std::cout << "Index out of range return empty position matrix: " << e.what() << "\n";
        extrMat = cv::Matx44f();
    }

    return extrMat;
}

cv::Matx44f cameraAPI::CameraPosition::get_camera_extrinsic_diff(int idx_from, int idx_to)
{
    cv::Matx44f extr_from, extr_to, ret_mx;

    try {
        extr_from = _camera_extrinsics.at(idx_from);
        extr_to = _camera_extrinsics.at(idx_to);

    } catch (const std::out_of_range &e) {
        std::cout << "Index out of range return empty extr matrix: " << e.what() << "\n";
        return cv::Matx44f();
    }

    cv::Matx33f rot_from(extr_from(0,0), extr_from(0,1), extr_from(0,2),
                         extr_from(1,0), extr_from(1,1), extr_from(1,2),
                         extr_from(2,0), extr_from(2,1), extr_from(2,2));
    cv::Matx33f rot_to(extr_to(0,0), extr_to(0,1), extr_to(0,2),
                       extr_to(1,0), extr_to(1,1), extr_to(1,2),
                       extr_to(2,0), extr_to(2,1), extr_to(2,2));
    cv::Matx13f translation({extr_to(0,3) - extr_from(0,3), extr_to(1,3)  - extr_from(1,3), extr_to(2,3) - extr_from(2,3)});

    cv::Matx33f rotation = get_rotation_difference(rot_from, rot_to);


    ret_mx = cv::Matx44f(rotation(0,0), rotation(0,1), rotation(0,2), translation(0),
                         rotation(1,0), rotation(1,1), rotation(1,2), translation(1),
                         rotation(2,0), rotation(2,1), rotation(2,2), translation(2),
                         0,                 0,                 0,                 1);

    return ret_mx;
}

cv::Matx44f cameraAPI::CameraPosition::get_relative_diff(int idx_from, int idx_to)
{
    cv::Matx44f extr_from, extr_to, ret_mx;

    try {
        extr_from = _camera_extrinsics.at(idx_from);
        extr_to = _camera_extrinsics.at(idx_to);

    } catch (const std::out_of_range &e) {
        std::cout << "Index out of range return empty extr matrix: " << e.what() << "\n";
        return cv::Matx44f();
    }

    cv::Matx33f rot_from(extr_from(0,0), extr_from(0,1), extr_from(0,2),
                         extr_from(1,0), extr_from(1,1), extr_from(1,2),
                         extr_from(2,0), extr_from(2,1), extr_from(2,2));
    cv::Matx33f rot_to(extr_to(0,0), extr_to(0,1), extr_to(0,2),
                       extr_to(1,0), extr_to(1,1), extr_to(1,2),
                       extr_to(2,0), extr_to(2,1), extr_to(2,2));

    // calculate position of the shift in relative
    cv::Matx31f relative_position = {extr_to(0,3) - extr_from(0,3), extr_to(1,3) - extr_from(1,3), extr_to(2,3) - extr_from(2,3)};
    relative_position = rot_from * relative_position;

    cv::Matx33f rotation = get_rotation_difference(rot_from, rot_to);

    ret_mx = cv::Matx44f(rotation(0,0), rotation(0,1), rotation(0,2), relative_position(0),
                         rotation(1,0), rotation(1,1), rotation(1,2), relative_position(1),
                         rotation(2,0), rotation(2,1), rotation(2,2), relative_position(2),
                         0,             0,             0,             1);

    return ret_mx;
}

cv::Matx33f cameraAPI::CameraPosition::get_rotation_difference(cv::Matx33f from, cv::Matx33f to)
{
    // assuming that to = R * from,
    // so R = to * inv(from)
    cv::Matx33f from_inverted;
    cv::invert(from, from_inverted);

    return to * from_inverted;
}

void cameraAPI::CameraPosition::clear()
{
    _camera_positions.clear();
    _camera_extrinsics.clear();
}

bool cameraAPI::CameraPosition::read(std::string filename)
{
    cv::FileStorage fs;
    if (!fs.open(filename, cv::FileStorage::READ)) {
        std::cout << filename << " cannot be opened.";
        return false;
    }

    int steps;
    fs["steps"] >> steps;
    if (steps == 0) {
        std::cout << "Warning. Steps in file are 0.\n";
        return false;
    }

    for (int i = 0; i < steps; ++i) {
        cv::Matx44f mat;
        fs["position" + std::to_string(i)] >> mat;

        _camera_extrinsics.push_back(mat);
    }

    return true;
}


// ================ HandCamera ===================

cameraAPI::HandCameraPosition::HandCameraPosition() :
    CameraPosition()
{
}

std::vector<double> cameraAPI::HandCameraPosition::move(double x, double y, double z)
{
    double input;
    std::cout << "Camera move requested. Please move camera by " << x << " m\n";
    std::cout << "Input actual translated value: ";
    std::cin >> input;
    std::cout << std::endl;

    _posX += input;

    // TODO rotation - get from user or calculate from images

    _camera_positions.push_back(get_current_camera_position());
    _camera_extrinsics.push_back(get_current_extrinsic_matrix());

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
    _acquisition_thread = std::thread(&CameraThread::_read, this);

}

void cameraAPI::CameraThread::stop()
{
    if (!_running) {
        std::cout << "Thread is already stopped.\n";
        return;
    }
    _running = false;
    _acquisition_thread.join();
}


cameraAPI::CameraThread::~CameraThread()
{
    if (_running)
        stop();
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

