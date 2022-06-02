#ifndef CAMERA_H
#define CAMERA_H

#include <vector>
#include <opencv2/videoio.hpp>
#include <queue>
#include <thread>
#include <atomic>
#include <safequeue.h>

namespace cameraAPI {

enum CameraType {
    Hand,
    Virtual
};


/*!
 * \brief The Camera class - base camera class for camera movement API
 * Position is in camera coordinates relative to the first position in initialization
 */
class CameraPosition {

public:
    CameraPosition();

    /*!
     * \brief init - initializes camera position based on given coordinates by calibration
     * \param position - vector of position containing {_posX, _posY, _posZ, _roll, _pitch, _yaw};
     */
    virtual void init(std::vector<double> position);

    /*!
     * \brief move - moves camera to given position and appends it in camera_positions history
     * \param x
     * \param y
     * \param z
     * \return vector with acquired position coordinates
     */
    virtual std::vector<double> move(double x, double y, double z) = 0;

    //getters
    double get_posX() { return _posX; }
    double get_posY() { return _posY; }
    double get_posZ() { return _posZ; }

    double get_pitch() { return _pitch; }
    double get_roll() { return _roll; }
    double get_yaw() { return _yaw; }

    virtual std::vector<double> get_current_position() { return {_posX, _posY, _posZ, _roll, _pitch, _yaw}; }
    virtual cv::Mat get_current_camera_position(); //returns C = - inv(R) * T matrix, position in world coordinates
    virtual cv::Matx44f get_current_extrinsic_matrix(); //returns [R|T] matrix

    virtual cv::Mat get_camera_position(int index);


protected:
    double _posX, _posY, _posZ;

    double _pitch, _roll, _yaw; //radians

    std::vector<cv::Mat> _camera_positions;
};


class HandCameraPosition : public CameraPosition {

/* Assuming camera movement is straight line
 *
 *          OBJECT
 *
 *          -->
 *  ----U-----------------------
 *  (camera)
 *
 *  So giving and getting the camera position is done by hand,
 *  only in one dimension. So main position change is translation with
 *  slight rotation also in one dimension. Although there is possibility
 *  that rotation in future will be calculated based on indicator in the scene.
 *
 */

public:
    HandCameraPosition();

    std::vector<double> move(double x, double y, double z) override;
    std::vector<double> move(double x);

};


/*!
 * \brief The CameraThread Singleton class - implements camera thread for VideoCapture
 * for capturing frames live (without buffer)
 */
class CameraThread {

private:
    std::atomic<bool> _running;
    int _device_id, _api_id;
    std::mutex _queue_mutex;

protected:
    CameraThread(int device_id, int api_id);
    ~CameraThread() {};
    void _read();

    cv::VideoCapture _camera;
    SafeQueue<cv::Mat> _frames_queue;
    std::unique_ptr<std::thread> _acquisition_thread;

public:

    CameraThread(CameraThread &other) = delete;

    void operator=(const CameraThread &) = delete;

    /*!
     * \brief GetInstance - static method controlling access to the singleton instance
     * On the first run creates object and places it into static field
     * \param device_id - VideoCapture camera deviceID
     * \param api_id - VideoCapture apiID
     * \return CameraThread signleton instance
     */
    static CameraThread& GetInstance(int device_id, int api_id);

    /*!
     * \brief GetInstance - static method controlling access to the singleton instance
     * On the first run creates object with default values
     * \return CameraThread signleton instance
     */
    static CameraThread& GetInstance();


    /*!
     * \brief read - returns live frame from the top of the queue, dropping all others in buffor
     * \return live camera frame
     */
    cv::Mat read();

    void stop();

};


inline CameraThread &CameraThread::GetInstance(int device_id, int api_id)
{
    static CameraThread instance(device_id, api_id);
    return instance;
}

inline CameraThread &CameraThread::GetInstance()
{
    int deviceID = 0;
    int apiID = cv::CAP_ANY;

    static CameraThread instance(deviceID, apiID);

    return instance;
}



}; // ~cameraAPI

#endif // CAMERA_H
