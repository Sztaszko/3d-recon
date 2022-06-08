#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "camera.h"
#include "settings.h"


class CameraCalibrator {
    public:
    CameraCalibrator() :
        flag(0)
      , _images_count(25)
    {
    };

    /*!
     * \brief getCalibImages - reading loop from acquisition thread of the
     * calibration chessboard. Saves captured images in the working directory
     * \return list of saved files of calibration chessboard
     */
    std::vector<std::string> getCalibImages();

    /*!
     * \brief addChessboardPoints - Open the chessboard images and extract corner points.
     * Fills imageCorners and objectCorners fields correspondingly.
     * Additionaly draws chessboard corners.
     * \param filelist - list of calib images files
     * \param boardSize - size of the calibration board
     * \return count of successes inserted corner points
     */
    int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize);

    /*!
     * \brief addPoints - append corresponding imageCorners and objectCorners to imagePoints and objectPoints
     * \param imageCorners
     * \param objectCorners
     */
    void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);

    /*!
     * \brief calibrate - calibrate camera based on objectPoints and imagePoints. Saves results to
     * cameraMatrix, distCoeffs, rvecs and tvecs.
     * \param imageSize
     * \return reprojection error
     */
    double calibrate(cv::Size &imageSize);


    // TODO add saving and reading calibrated camera params
    void saveCameraParams(std::string filename);

    void readCameraParams(std::string filename);

    // TODO add remove distorsion from calibration images

    // Getters
    cv::Mat getCameraMatrix() { return cameraMatrix; }
    cv::Mat getDistCoeffs()   { return distCoeffs; }
    std::vector<std::vector<cv::Point3f>> get_objectPoints() { return objectPoints; }
    std::vector<std::vector<cv::Point2f>> get_imagePoints() { return imagePoints; }
    std::vector<cv::Mat> get_rvecs() { return rvecs; }
    std::vector<cv::Mat> get_tvecs() { return tvecs; }
    std::vector<std::string> get_chessboard_files() { return _chessboard_files; };
    double get_reprojection_err() { return reprojection_err; }

private:
    // flag to specify how calibration is done
    int flag;

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::string> _chessboard_files;

    // output Matrices
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    std::vector<cv::Mat> rvecs, tvecs;

    double reprojection_err;

    int _images_count;
    cv::Size _board_size;
};

#endif // CAMERACALIBRATOR_H
