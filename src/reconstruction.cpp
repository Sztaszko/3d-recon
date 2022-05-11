#include "reconstruction.h"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/sfm.hpp>
#include <opencv2/core.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

void Reconstructor::reconstruct()
{

}

void Reconstructor::match_points(cv::Mat image1, cv::Mat image2,
                                 std::vector<cv::Point2f> &points1,
                                 std::vector<cv::Point2f> &points2)
{
    cv::Mat matchImage;
    std::vector<std::vector<cv::Point2f>> ret_arr;

    // vector of keypoints and descriptors
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors1, descriptors2;

    // Construction of the SIFT feature detector
    cv::Ptr<cv::Feature2D> ptrFeature2D = cv::SIFT::create(10000);

    // Detection of the SIFT features and associated descriptors
    ptrFeature2D->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
    ptrFeature2D->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);

    // Match the two image descriptors
    // Construction of the matcher with crosscheck
    cv::BFMatcher matcher(cv::NORM_L2, true);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    cv::namedWindow("img1");
    cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, matchImage,
                    cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imwrite("matches.jpg", matchImage);


    for (std::vector<cv::DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it) {
      // Get the position of left keypoints
      float x = keypoints1[it->queryIdx].pt.x;
      float y = keypoints1[it->queryIdx].pt.y;
      points1.push_back(cv::Point2f(x, y));
      // Get the position of right keypoints
      x = keypoints2[it->trainIdx].pt.x;
      y = keypoints2[it->trainIdx].pt.y;
      points2.push_back(cv::Point2f(x, y));
    }

    return ret_arr;
}

cv::Vec3d Reconstructor::triangulate(const cv::Mat &p1, const cv::Mat &p2, const cv::Vec2d &u1, const cv::Vec2d &u2) {

  // system of equations assuming image=[u,v] and X=[x,y,z,1]
  // from u(p3.X)= p1.X and v(p3.X)=p2.X
  cv::Matx43d A(u1(0)*p1.at<double>(2, 0) - p1.at<double>(0, 0),
  u1(0)*p1.at<double>(2, 1) - p1.at<double>(0, 1),
  u1(0)*p1.at<double>(2, 2) - p1.at<double>(0, 2),
  u1(1)*p1.at<double>(2, 0) - p1.at<double>(1, 0),
  u1(1)*p1.at<double>(2, 1) - p1.at<double>(1, 1),
  u1(1)*p1.at<double>(2, 2) - p1.at<double>(1, 2),
  u2(0)*p2.at<double>(2, 0) - p2.at<double>(0, 0),
  u2(0)*p2.at<double>(2, 1) - p2.at<double>(0, 1),
  u2(0)*p2.at<double>(2, 2) - p2.at<double>(0, 2),
  u2(1)*p2.at<double>(2, 0) - p2.at<double>(1, 0),
  u2(1)*p2.at<double>(2, 1) - p2.at<double>(1, 1),
  u2(1)*p2.at<double>(2, 2) - p2.at<double>(1, 2));

  cv::Matx41d B(p1.at<double>(0, 3) - u1(0)*p1.at<double>(2,3),
                p1.at<double>(1, 3) - u1(1)*p1.at<double>(2,3),
                p2.at<double>(0, 3) - u2(0)*p2.at<double>(2,3),
                p2.at<double>(1, 3) - u2(1)*p2.at<double>(2,3));

  // X contains the 3D coordinate of the reconstructed point
  cv::Vec3d X;
  // solve AX=B
  cv::solve(A, B, X, cv::DECOMP_SVD);
  return X;
}

// triangulate a vector of image points
void Reconstructor::triangulate(const cv::Mat &p1, const cv::Mat &p2, const std::vector<cv::Vec2d> &pts1, const std::vector<cv::Vec2d> &pts2, std::vector<cv::Vec3d> &pts3D) {

  for (int i = 0; i < pts1.size(); i++) {

    pts3D.push_back(triangulate(p1, p2, pts1[i], pts2[i]));
  }
}
