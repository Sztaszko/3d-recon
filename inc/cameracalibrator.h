#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "camera.h"


class CameraCalibrator {
    public:
    CameraCalibrator() :
        flag(0),
        mustInitUndistort(true)
    {};

    // Get the calibration images
    std::vector<std::string> getCalibImages(cameraAPI::Camera& cam);

    // Open the chessboard images and extract corner points
    int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize);

    // Add scene points and corresponding image points
    void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);

    // Calibrate the camera
    double calibrate(cv::Size &imageSize);

    // Remove distortion in an image (after calibration)
    cv::Mat remap(const cv::Mat &image);

    // Getters
    cv::Mat getCameraMatrix() { return cameraMatrix; }
    cv::Mat getDistCoeffs()   { return distCoeffs; }
    std::vector<std::vector<cv::Point3f>> get_objectPoints() { return objectPoints; }
    std::vector<std::vector<cv::Point2f>> get_imagePoints() { return imagePoints; }
    cv::Mat get_cameraMatrix() {return cameraMatrix; };
    cv::Mat get_distCoeffs() { return distCoeffs; }
    std::vector<cv::Mat> get_rvecs() { return rvecs; }
    std::vector<cv::Mat> get_tvecs() { return tvecs; }
    double get_reprojection_err() { return reprojection_err; }

private:
    // flag to specify how calibration is done
    int flag;

    // used in image undistortion
    cv::Mat map1, map2;
    bool mustInitUndistort;

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;

    // output Matrices
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    std::vector<cv::Mat> rvecs, tvecs;

    double reprojection_err;
};

#endif // CAMERACALIBRATOR_H
