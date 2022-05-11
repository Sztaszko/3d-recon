#include "reconstruction.h"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/sfm.hpp>
#include <opencv2/core.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>


Reconstructor::Reconstructor()
{

}

void Reconstructor::init()
{
    chessboard_files = {"boards/1.jpg", "boards/2.jpg",
                       "boards/3.jpg","boards/4.jpg",
                       "boards/5.jpg","boards/6.jpg",
                       "boards/7.jpg","boards/8.jpg",
                       "boards/9.jpg","boards/10.jpg",
                       "boards/11.jpg","boards/12.jpg",
                       "boards/13.jpg","boards/14.jpg",
                       "boards/15.jpg","boards/16.jpg",
                       "boards/17.jpg","boards/18.jpg",
                       "boards/19.jpg","boards/20.jpg",
                       "boards/21.jpg","boards/22.jpg",
                       "boards/23.jpg","boards/24.jpg",
                       "boards/25.jpg"};
}

void Reconstructor::reconstruct(cv::Mat image1, cv::Mat image2)
{
    cv::Size board_size(7,7);

    cal.addChessboardPoints(chessboard_files, board_size);

    cv::Mat img = cv::imread(chessboard_files[0]);
    cv::Size img_size = img.size();

    cal.calibrate(img_size);
    std::cout << cal.get_cameraMatrix() << std::endl;


    std::vector<cv::Point2f> points1, points2;
    match_points(image1, image2, points1, points2);

    //TODO this segment is meant to be replaced by getting translation and rotation from camera move API:
    // ===========================

    // Find the essential between image 1 and image 2
    cv::Mat inliers;
    cv::Mat essential = cv::findEssentialMat(points1, points2, cal.get_cameraMatrix(), cv::RANSAC, 0.9, 1.0, inliers);

    std::cout << essential << std::endl;

    // recover relative camera pose from essential matrix
    cv::Mat rotation, translation;
    cv::recoverPose(essential, points1, points2, cal.getCameraMatrix(), rotation, translation, inliers);
    std::cout << rotation << std::endl;
    std::cout << translation << std::endl;
    // ==========================

    // compose projection matrix from R,T
    std::pair<cv::Mat, cv::Mat> projections = get_projection_mats(rotation, translation);
    cv::Mat projection1 = projections.first;
    cv::Mat projection2 = projections.second;


    // to contain the inliers
    std::vector<cv::Vec2d> inlierPts1;
    std::vector<cv::Vec2d> inlierPts2;

    // create inliers input point vector for triangulation
    int j(0);
    for (int i = 0; i < inliers.rows; i++) {
      if (inliers.at<uchar>(i)) {
        inlierPts1.push_back(cv::Vec2d(points1[i].x, points1[i].y));
        inlierPts2.push_back(cv::Vec2d(points2[i].x, points2[i].y));
      }
    }

    // undistort and normalize the image points
    std::vector<cv::Vec2d> points1u;
    cv::undistortPoints(inlierPts1, points1u, cal.get_cameraMatrix(), cal.get_distCoeffs());
    std::vector<cv::Vec2d> points2u;
    cv::undistortPoints(inlierPts2, points2u, cal.get_cameraMatrix(), cal.get_distCoeffs());

    // Triangulation
    std::vector<cv::Vec3d> points3D;
    triangulate(projection1, projection2, points1u, points2u, points3D);

    std::cout<<"3D points :"<<points3D.size()<<std::endl;

    cv::viz::Viz3d window; //creating a Viz window

    //Displaying the Coordinate Origin (0,0,0)
    window.showWidget("coordinate", cv::viz::WCoordinateSystem());

    window.setBackgroundColor(cv::viz::Color::black());

    //Displaying the 3D points in green
    window.showWidget("points", cv::viz::WCloud(points3D, cv::viz::Color::green()));
    window.spin();
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

}

std::pair<cv::Mat, cv::Mat>  Reconstructor::get_projection_mats(cv::Mat rotation, cv::Mat translation)
{
    std::pair<cv::Mat, cv::Mat> ret_pair;

    // compose projection matrix from R,T
    cv::Mat projection2(3, 4, CV_64F); // the 3x4 projection matrix
    rotation.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
    translation.copyTo(projection2.colRange(3, 4));

    // compose generic projection matrix
    cv::Mat projection1(3, 4, CV_64F, 0.); // the 3x4 projection matrix
    cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
    diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));

    ret_pair.first = projection1;
    ret_pair.second = projection2;

    return ret_pair;
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
