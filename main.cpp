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
#include "camera.h"

using namespace std;
using namespace cv;
using namespace cv::sfm;


static void help() {
  cout
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
      files.push_back(path_to_file+string("/") + line_str);
  }
  return 1;
}


int main(){

    cameraAPI::HandCamera camera;
    int deviceID = 0;
    int apiID = cv::CAP_ANY;
    if (!camera.open(deviceID, apiID)) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    Reconstructor reconstructor;
    reconstructor.init(camera); //calibration

    cv::Mat image1 = cv::imread("imR.png");
    cv::Mat image2 = cv::imread("imL.png");

    std::vector<cv::Vec3d> points3D = reconstructor.reconstruct(image1, image2);

    // visualize
    cv::viz::Viz3d window; //creating a Viz window

    //Displaying the Coordinate Origin (0,0,0)
    window.showWidget("coordinate", cv::viz::WCoordinateSystem());

    window.setBackgroundColor(cv::viz::Color::black());

    //Displaying the 3D points in green
    window.showWidget("points", cv::viz::WCloud(points3D, cv::viz::Color::green()));
    window.spin();

}
