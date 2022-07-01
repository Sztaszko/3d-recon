#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include <opencv2/core/core.hpp>
#include "cameracalibrator.h"
#include "camera.h"


class Reconstructor {
public:
    Reconstructor();

    friend class SandBox; //TODO delete, only for testing and trials
    /*!
     * \brief init - perform calibration using calibrator cal
     * \return true - calibration successful, false - calibration failed
     */
    bool init();

    /*!
     * \brief init - overloaded init() loading params from file
     * \param camera_params_file - absolute path to xml or yml file with calibration parameters
     * \return true - if reading successful, false - if reading failed
     */
    bool init(std::string camera_params_file);

    std::vector<cv::Vec3d> reconstruct(std::vector<std::string> filenames, cameraAPI::CameraPosition& postisions);

    std::vector<cv::Mat> reconstruct_opencv(std::vector<std::string> filenames);

    // getters
    std::vector<cv::Vec3d> get_points3D() { return points3D; }
    CameraCalibrator* get_calibrator() { return &cal; }

private:
    void match_points(cv::Mat image1, cv::Mat image2,
                      std::vector<cv::Point2f> &points1,
                      std::vector<cv::Point2f> &points2);

    void create_inliers(std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2,
                        std::vector<cv::Vec2d> &inlier1, std::vector<cv::Vec2d> &inlier2, cv::Mat inliers);


    std::pair<cv::Mat, cv::Mat> get_projection_mats(cv::Mat rotation, cv::Mat translation);

    cv::Vec3d triangulate(const cv::Mat &p1, const cv::Mat &p2, const cv::Vec2d &u1, const cv::Vec2d &u2);
    void triangulate(const cv::Mat &p1, const cv::Mat &p2,
                     const std::vector<cv::Vec2d> &pts1,
                     const std::vector<cv::Vec2d> &pts2,
                     std::vector<cv::Vec3d> &pts3D);


    CameraCalibrator cal;
    std::vector<std::string> chessboard_files;
    std::vector<cv::Vec3d> points3D;
};

class SandBox {

public:
    Reconstructor reconstructor;

    void run() {
        std::cout << "camMx:" << reconstructor.get_calibrator()->getCameraMatrix();
        std::cout << "distCoef: "<< reconstructor.get_calibrator()->getDistCoeffs();

        cv::Mat image1 = cv::imread("/home/skaczmarski/projects/3d-recon/build-3d-recon-Desktop_Qt_5_15_2_GCC_64bit-Debug/0_13.28_0.0.png");
        cv::Mat image2 = cv::imread("/home/skaczmarski/projects/3d-recon/build-3d-recon-Desktop_Qt_5_15_2_GCC_64bit-Debug/1_12.63_-4.1.png");

        // image points
        std::vector<cv::Point2f> points1, points2;
//        reconstructor.match_points(image1, image2, points1, points2);


        cv::Mat img1_undist;
        cv::Mat img2_undist;
        cv::undistort(image1, img1_undist,
                      reconstructor.get_calibrator()->getCameraMatrix(),
                      reconstructor.get_calibrator()->getDistCoeffs());
        cv::undistort(image2, img2_undist,
                      reconstructor.get_calibrator()->getCameraMatrix(),
                      reconstructor.get_calibrator()->getDistCoeffs());
        reconstructor.match_points(img1_undist, img2_undist, points1, points2);

    }

};


#endif // RECONSTRUCTION_H
