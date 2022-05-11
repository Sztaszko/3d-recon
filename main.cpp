#define CERES_FOUND 1

#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <fstream>
#include "reconstruction.h"
#include "cameracalibrator.h"

using namespace std;
using namespace cv;
using namespace cv::sfm;


static void help() {
  cout
      << "\n------------------------------------------------------------------------------------\n"
      << " This program shows the multiview reconstruction capabilities in the \n"
      << " OpenCV Structure From Motion (SFM) module.\n"
      << " It reconstruct a scene from a set of 2D images \n"
      << " Usage:\n"
      << "        example_sfm_scene_reconstruction <path_to_file> <f> <cx> <cy>\n"
      << " where: path_to_file is the file absolute path into your system which contains\n"
      << "        the list of images to use for reconstruction. \n"
      << "        f  is the focal length in pixels. \n"
      << "        cx is the image principal point x coordinates in pixels. \n"
      << "        cy is the image principal point y coordinates in pixels. \n"
      << "------------------------------------------------------------------------------------\n\n"
      << endl;
}
static int getdir(const string _filename, vector<String> &files)
{
  ifstream myfile(_filename.c_str());
  if (!myfile.is_open()) {
    cout << "Unable to read file: " << _filename << endl;
    exit(0);
  } else {;
    size_t found = _filename.find_last_of("/\\");
    string line_str, path_to_file = _filename.substr(0, found);
    while ( getline(myfile, line_str) )
      files.push_back(path_to_file+string("/")+line_str);
  }
  return 1;
}



int main(){

  cout<<"compiled"<<endl;

  const std::vector<std::string> files = {"boards/1.jpg", "boards/2.jpg","boards/3.jpg","boards/4.jpg","boards/5.jpg","boards/6.jpg","boards/7.jpg","boards/8.jpg","boards/9.jpg","boards/10.jpg","boards/11.jpg","boards/12.jpg","boards/13.jpg","boards/14.jpg","boards/15.jpg","boards/16.jpg","boards/17.jpg","boards/18.jpg","boards/19.jpg","boards/20.jpg","boards/21.jpg","boards/22.jpg","boards/23.jpg","boards/24.jpg","boards/25.jpg"};
  cv::Size board_size(7,7);

  CameraCalibrator cal;
  cal.addChessboardPoints(files, board_size);

  Reconstructor reconstructor;

  cv::Mat img = cv::imread(files[0]);

  cv::Size img_size = img.size();
  cal.calibrate(img_size);
  cout << cal.get_cameraMatrix() << endl;


  cv::Mat image1 = cv::imread("imR.png");
  cv::Mat image2 = cv::imread("imL.png");

  std::vector<cv::Point2f> points1, points2;
  reconstructor.match_points(image1, image2, points1, points2);

  //TODO this segment is meant to be replaced by getting translation and rotation from camera move API:
  // ===========================

  // Find the essential between image 1 and image 2
  cv::Mat inliers;
  cv::Mat essential = cv::findEssentialMat(points1, points2, cal.get_cameraMatrix(), cv::RANSAC, 0.9, 1.0, inliers);

  cout<<essential<<endl;

  // recover relative camera pose from essential matrix
  cv::Mat rotation, translation;
  cv::recoverPose(essential, points1, points2, cal.getCameraMatrix(), rotation, translation, inliers);
  cout<<rotation<<endl;
  cout<<translation<<endl;

  // ==========================

  // compose projection matrix from R,T
  cv::Mat projection2(3, 4, CV_64F); // the 3x4 projection matrix
  rotation.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
  translation.copyTo(projection2.colRange(3, 4));

  // compose generic projection matrix
  cv::Mat projection1(3, 4, CV_64F, 0.); // the 3x4 projection matrix
  cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
  diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));

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
  reconstructor.triangulate(projection1, projection2, points1u, points2u, points3D);

  cout<<"3D points :"<<points3D.size()<<endl;

  viz::Viz3d window; //creating a Viz window

  //Displaying the Coordinate Origin (0,0,0)
  window.showWidget("coordinate", viz::WCoordinateSystem());

  window.setBackgroundColor(cv::viz::Color::black());

  //Displaying the 3D points in green
  window.showWidget("points", viz::WCloud(points3D, viz::Color::green()));
  window.spin();


}
