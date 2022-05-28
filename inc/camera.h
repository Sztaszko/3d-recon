#ifndef CAMERA_H
#define CAMERA_H

#include <vector>
#include <opencv2/videoio.hpp>

namespace cameraAPI {

/*!
 * \brief The Camera class - base camera class for camera movement API
 * Position is in camera coordinates relative to the first position in initialization
 */
class Camera : public cv::VideoCapture {

public:
    Camera();

    /*!
     * \brief init - initializes camera position based on given coordinates by calibration
     * \param position - vector of position containing {_posX, _posY, _posZ, _roll, _pitch, _yaw};
     */
    virtual void init(std::vector<double> position);

    /*!
     * \brief move - moves camera to given position
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

    virtual std::vector<double> get_position() { return {_posX, _posY, _posZ, _roll, _pitch, _yaw}; }
    virtual cv::Mat get_camera_position(); //returns C = - inv(R) * T matrix, position in world coordinates
    virtual cv::Matx44f get_extrinsic_matrix(); //returns [R|T] matrix


protected:
    double _posX, _posY, _posZ;

    double _pitch, _roll, _yaw; //radians

};


class HandCamera : public Camera {

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
    HandCamera();

    std::vector<double> move(double x, double y, double z) override;
    std::vector<double> move(double x);

};


}; // ~cameraAPI

#endif // CAMERA_H
