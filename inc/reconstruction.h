#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include <opencv2/core/core.hpp>
#include "cameracalibrator.h"
#include "camera.h"

class Reconstructor {
public:
    Reconstructor();

    bool init(cameraAPI::Camera& cam);

    std::vector<cv::Vec3d> reconstruct(cv::Mat image1, cv::Mat image2);

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


private:
    CameraCalibrator cal;
    std::vector<std::string> chessboard_files;
    std::vector<cv::Vec3d> points3D;
};


#endif // RECONSTRUCTION_H
