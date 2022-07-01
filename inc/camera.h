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

    /*!
     * \brief get_current_position
     * \return current position as a vector
     */
    virtual std::vector<double> get_current_position() { return {_posX, _posY, _posZ, _roll, _pitch, _yaw}; }

    /*!
     * \brief get_current_camera_position
     * \return C = - inv(R) * T matrix, position in world coordinates
     */
    virtual cv::Mat get_current_camera_position();

    /*!
     * \brief get_current_extrinsic_matrix
     * \return [R|T] matrix of current camera postion
     */
    virtual cv::Matx44f get_current_extrinsic_matrix();

    /*!
     * \brief get_camera_position - access to the position at given index in the past
     * \param index - position index from the past
     * \return camera position in world coordinates
     */
    virtual cv::Mat get_camera_position(int index);

    /*!
     * \brief get_camera_extrinsic
     * \param index
     * \return
     */
    virtual cv::Matx44f get_camera_extrinsic(int index);


    virtual cv::Matx44f get_camera_extrinsic_diff(int idx_from, int idx_to);

    /*!
     * \brief get_rotation_difference - calculates difference rotation between from and to
     * \param from
     * \param to
     * \return rotation between given matrices
     */
    virtual cv::Matx33f get_rotation_difference(cv::Matx33f from, cv::Matx33f to);

    virtual void clear();

    /*!
     * \brief read - reads camera extrinsics from file
     * \param filename - file path saved in acquisition
     * \return true - if reading was successful, false - if error occured
     */
    virtual bool read(std::string filename);

    //getters
    double get_posX() { return _posX; }
    double get_posY() { return _posY; }
    double get_posZ() { return _posZ; }

    double get_pitch() { return _pitch; }
    double get_roll() { return _roll; }
    double get_yaw() { return _yaw; }

    std::vector<cv::Matx44f> get_camera_extrinsics() { return _camera_extrinsics; }

protected:
    double _posX, _posY, _posZ;

    double _pitch, _roll, _yaw; //radians

    /*!
     * \brief _camera_positions - holds global camera positions matrices
     * saved every move performed
     */
    std::vector<cv::Mat> _camera_positions;

    /*!
     * \brief _camera_extrinsics - holds camera extrinsic matrices in relative
     * translation and rotation to the first camera position
     */
    std::vector<cv::Matx44f> _camera_extrinsics;
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
    CameraThread() {};
    ~CameraThread();
    void _read();

    cv::VideoCapture _camera;
    SafeQueue<cv::Mat> _frames_queue;
    std::thread _acquisition_thread;

public:

    CameraThread(CameraThread &other) = delete;

    void operator=(const CameraThread &) = delete;

    /*!
     * \brief GetInstance - static method controlling access to the singleton instance
     * On the first run creates object with set values in init()
     * \return CameraThread signleton instance
     */
    static CameraThread& GetInstance();


    /*!
     * \brief read - returns live frame from the top of the queue, dropping all others in buffor
     * \return live camera frame
     */
    cv::Mat read();

    /*!
     * \brief init - initializes values for video capture api
     * \param device_id - camera id
     * \param api_id - id as in VideoCapture
     */
    void init(int device_id, int api_id);

    /*!
     * \brief start - starts acquisition thread
     */
    void start();

    /*!
     * \brief stop - stops thread and waits by join() method
     */
    void stop();

};


inline CameraThread &CameraThread::GetInstance()
{
    static CameraThread instance;
    return instance;
}



}; // ~cameraAPI

#endif // CAMERA_H
