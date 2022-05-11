#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include <opencv2/core/core.hpp>
#include "cameracalibrator.h"

class Reconstructor {
public:
    Reconstructor();

    void init();

    void reconstruct(cv::Mat image1, cv::Mat image2);

    void match_points(cv::Mat image1, cv::Mat image2,
                      std::vector<cv::Point2f> &points1,
                      std::vector<cv::Point2f> &points2);


    std::pair<cv::Mat, cv::Mat> get_projection_mats(cv::Mat rotation, cv::Mat translation);

    cv::Vec3d triangulate(const cv::Mat &p1, const cv::Mat &p2, const cv::Vec2d &u1, const cv::Vec2d &u2);
    void triangulate(const cv::Mat &p1, const cv::Mat &p2,
                     const std::vector<cv::Vec2d> &pts1,
                     const std::vector<cv::Vec2d> &pts2,
                     std::vector<cv::Vec3d> &pts3D);

private:
    std::vector<std::string> chessboard_files;
    CameraCalibrator cal;
};


#endif // RECONSTRUCTION_H
