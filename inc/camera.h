#ifndef CAMERA_H
#define CAMERA_H

#include <vector>

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
    virtual std::vector<double> move(double x, double y, double z);

    //getters
    double get_posX() { return _posX; }
    double get_posY() { return _posY; }
    double get_posZ() { return _posZ; }


private:

    double _posX, _posY, _posZ;

};

#endif // CAMERA_H
