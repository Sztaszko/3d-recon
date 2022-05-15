#ifndef CAMERA_H
#define CAMERA_H

#include <vector>

namespace cameraAPI {

/*!
 * \brief The Camera class - base camera class for camera movement API
 */
class Camera {

public:
    Camera();

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


protected:

    double _posX, _posY, _posZ;

};


class HandCamera : public Camera {

public:
    HandCamera();

    std::vector<double> move(double x, double y, double z) override;

    std::vector<double> move(int alpha);

};


}; // ~cameraAPI

#endif // CAMERA_H
