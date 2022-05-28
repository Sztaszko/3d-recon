#define CERES_FOUND 1

#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/core.hpp>
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <fstream>
#include "reconstruction.h"
#include "cameracalibrator.h"
#include "camera.h"



static void help() {
  std::cout
      << "\n------------------------------------------------------------------------------------\n"
      << " This program performs multiview sfm reconstruction \n"
      << " Usage:\n"
      << "        example_sfm_scene_reconstruction <path_to_file> <f> <cx> <cy>\n"
      << " where: path_to_file is the file absolute path into your system which contains\n"
      << "        the list of images to use for reconstruction. \n"
      << "        f  is the focal length in pixels. \n"
      << "        cx is the image principal point x coordinates in pixels. \n"
      << "        cy is the image principal point y coordinates in pixels. \n"
      << "------------------------------------------------------------------------------------\n\n"
      << std::endl;
}
static int getdir(const std::string _filename, std::vector<cv::String> &files)
{
  std::ifstream myfile(_filename.c_str());
  if (!myfile.is_open()) {
    std::cout << "Unable to read file: " << _filename << std::endl;
    exit(0);
  } else {;
    size_t found = _filename.find_last_of("/\\");
    std::string line_str, path_to_file = _filename.substr(0, found);
    while ( getline(myfile, line_str) )
      files.push_back(path_to_file + std::string("/") + line_str);
  }
  return 1;
}


int main(int argc, char *argv[]){

//    const cv::String keys =
//                "{help h usage ? |           | print this message            }"
//                "{@settings      |default.xml| input setting file            }";

//    cv::CommandLineParser parser(argc, argv, keys);
//    parser.about("This is structure from motion pipeline implementation with known position of the camera.\n"
//                 "Usage: 3d-recon [configuration file]");



    cameraAPI::HandCamera camera;
    int deviceID = 0;
    int apiID = cv::CAP_ANY;

    if (!camera.open(deviceID, apiID)) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    Reconstructor reconstructor;

    //calibration
    if (!reconstructor.init(camera)) {
        return -1;
    }

    // TODO get it from command line parser
    int x_distance = 10; // 10 meters
    double interval = 0.5; // meters

    int steps = static_cast<int>(static_cast<double>(x_distance)/interval);
    std::cout << "Going to move camera " << steps << " times by " << interval << " m.\n";

    // TODO get positions and iterating add interval to camera matrix
    CameraCalibrator* calibration = reconstructor.get_calibrator();

    int j = 0;
//    for (const auto &pos : positions) {
//        ++j;
//        std::cout << j << ". " << pos << "\n";
//    }

    cv::Mat cameraPosition;

    std::vector<cv::Mat> rvecs = calibration->get_rvecs();
    std::vector<cv::Mat> tvecs = calibration->get_tvecs();

    for (int i = 0; i < rvecs.size(); ++i) {
        cv::Mat rotMat;
        cv::Rodrigues(rvecs[i], rotMat);
        cv::transpose(rotMat, rotMat);
        cameraPosition = -rotMat * tvecs[i];

        std::cout << i+1 << ". Camera position: " << cameraPosition << "\n";

    }

    std::vector<std::string> chessboard_files = calibration->get_chessboard_files();

    cv::Mat img1 = cv::imread(chessboard_files[0]);
    cv::Mat img2 = cv::imread(chessboard_files[1]);

    reconstructor.reconstruct(img1, img2);


//    for (int i = 0; i < x_distance; )



//    cv::Mat image1 = cv::imread("imR.png");
//    cv::Mat image2 = cv::imread("imL.png");

//    std::vector<cv::Vec3d> points3D = reconstructor.reconstruct(image1, image2);

//    // visualize
//    cv::viz::Viz3d window; //creating a Viz window

//    //Displaying the Coordinate Origin (0,0,0)
//    window.showWidget("coordinate", cv::viz::WCoordinateSystem());

//    window.setBackgroundColor(cv::viz::Color::black());

//    //Displaying the 3D points in green
//    window.showWidget("points", cv::viz::WCloud(points3D, cv::viz::Color::green()));
//    window.spin();

}
