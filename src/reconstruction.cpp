#include "reconstruction.h"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/sfm.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

Reconstructor::Reconstructor()
{

}

/* TODO that's how whole pipeline should look like:
 * init() - camera calibration, get camera matrix
 *
 * get camera Matrix from calibrator and extract extrinsic matrix [R|T]
 *
 * Perform first movement scan - giving and saving positions of camera to a vector
 * for i in first_scan_positions:
 * std::vector<cv::Vec3d> positions.push_back(camera.move(i))
 *
 * Explore saved dataset and find the best 2 matching pictures
 * Remove outliers etc.
 * Undistort and triangulate 3D points from these 2 pictures
 * Save 3D points to a points cloud
 *
 * Iterate over all begginning pictures:
 *  Calculate camera position based on previously gained coordinates.
 *  Find correspondences and points visible by both pictures.
 *  Triangulate and add new points.
 *  Bundle adjustment
 *
 * When finished apply feedback algorythm  -> find where position where is less points and
 * find camera position for increasing theirs amount.
 *
 * Move camera to given position and take pictures. Find nearest previously taken pictures
 * and match points.
 * Traingulate, add points to 3D cloud and bundle adjust.
 *
 * Repeat until requested precision is completed or made no progress.
 *
 */


bool Reconstructor::init()
{

    std::vector<std::string> chessboard_files = cal.getCalibImages();
    cv::Size board_size(9,6);

    if (chessboard_files.empty()) {
        std::cerr << "Couldnt get chessboard images. Aborting";
        return false;
    }

    if (!cal.addChessboardPoints(chessboard_files, board_size)) {
        return false;
    }

    cv::Mat img = cv::imread(chessboard_files[0]);
    cv::Size img_size = img.size();

    cal.calibrate(img_size);
    cal.saveCameraParams("cameraParams.xml");
    std::cout << "Camera matrix: " << cal.getCameraMatrix() << std::endl;

    return true;

}

std::vector<cv::Vec3d> Reconstructor::reconstruct(std::vector<std::string> filenames,
                                                  cameraAPI::CameraPosition& positions)
{
    if (filenames.size() < 3) {
        std::cout << "There is less than 3 images. It is not enough to reconstruction. Aborting.\n";
        return points3D;
    }

    if (filenames.size() != positions.get_camera_extrinsics().size()) {
        // TODO maybe calculate missing positions by PnP
        std::cout << "Images and positions sizes are different. Aborting.\n";
        return points3D;
    }

    // TODO find the best match between images in the begginning


    for (int i = 0; i < filenames.size() - 1; ++i) {

        cv::Mat image1 = cv::imread(filenames.at(i));
        cv::Mat image2 = cv::imread(filenames.at(i + 1));

        // image points
        std::vector<cv::Point2f> points1, points2;

        match_points(image1, image2, points1, points2);

        cv::Mat inliers;
        cv::Mat rotation, translation;

        // get the inliers after replacing code above
        cv::Mat affine_transformation = cv::estimateAffine2D(points1, points2, inliers);

        cv::Matx44f position = positions.get_camera_extrinsic(0);

        rotation = cv::Mat(cv::Matx33f( position(0,0), position(0,1), position(0,2),
                                        position(1,0), position(1,1), position(1,2),
                                        position(2,0), position(2,1), position(2,2)), true);
        translation = cv::Mat({position(0,3), position(1,3), position(2,3)});

        // debug
        std::cout << "rotation from positions: " << rotation << "\n";
        std::cout << "translation from positions : " << translation << "\n";

        //TODO this segment is meant to be replaced by getting translation, rotation and inliers from camera move API:
        // ===========================

        // Find the essential between image 1 and image 2
        cv::Mat essential = cv::findEssentialMat(points1, points2, cal.getCameraMatrix(), cv::RANSAC, 0.9, 1.0, inliers);

        std::cout << "Essential matrix: " << essential << std::endl;

        // recover relative camera pose from essential matrix
        cv::recoverPose(essential, points1, points2, cal.getCameraMatrix(), rotation, translation, inliers);
        std::cout << "rotation from essential: " << rotation << std::endl;
        std::cout << "translation from essential: " << translation << std::endl;

        // ==========================



        // compose projection matrix from R,T
        std::pair<cv::Mat, cv::Mat> projections = get_projection_mats(rotation, translation);
        cv::Mat projection1 = projections.first;
        cv::Mat projection2 = projections.second;


        // to contain the inliers
        std::vector<cv::Vec2d> inlierPts1;
        std::vector<cv::Vec2d> inlierPts2;

        create_inliers(points1, points2, inlierPts1, inlierPts2, inliers);


        // undistort and normalize the image points
        std::vector<cv::Vec2d> points1u;
        cv::undistortPoints(inlierPts1, points1u, cal.getCameraMatrix(), cal.getDistCoeffs());
        std::vector<cv::Vec2d> points2u;
        cv::undistortPoints(inlierPts2, points2u, cal.getCameraMatrix(), cal.getDistCoeffs());

        // Triangulation
        triangulate(projection1, projection2, points1u, points2u, points3D);

    }

    std::cout<<"3D points :" << points3D.size() << std::endl;

    // TODO make bundle andjustment

    return points3D;
}

void Reconstructor::match_points(cv::Mat image1, cv::Mat image2,
                                 std::vector<cv::Point2f> &points1,
                                 std::vector<cv::Point2f> &points2)
{
    cv::Mat matchImage;

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

void Reconstructor::create_inliers(std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2,
                                   std::vector<cv::Vec2d> &inlierPts1, std::vector<cv::Vec2d> &inlierPts2, cv::Mat inliers)
{
    // create inliers input point vector for triangulation
    int j(0);
    for (int i = 0; i < inliers.rows; i++) {
      if (inliers.at<uchar>(i)) {
        inlierPts1.push_back(cv::Vec2d(points1[i].x, points1[i].y));
        inlierPts2.push_back(cv::Vec2d(points2[i].x, points2[i].y));
      }
    }
}

std::pair<cv::Mat, cv::Mat> Reconstructor::get_projection_mats(cv::Mat rotation, cv::Mat translation)
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
