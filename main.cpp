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
#include <unistd.h>
#include <locale>


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

std::string get_exec_path() {
    // get executing path
    char pBuf[256];
    size_t len = sizeof(pBuf);

    int bytes = MIN(readlink("/proc/self/exe", pBuf, len), len - 1);
    if(bytes >= 0)
        pBuf[bytes] = '\0';

    std::string path(pBuf);
    return path;
}


int main(int argc, char *argv[]){

//    const cv::String keys =
//                "{help h usage ? |           | print this message            }"
//                "{@settings      |default.xml| input setting file            }";

//    cv::CommandLineParser parser(argc, argv, keys);
//    parser.about("This is structure from motion pipeline implementation with known position of the camera.\n"
//                 "Usage: 3d-recon [configuration file]");

    cameraAPI::HandCameraPosition cameraPositions;


    int deviceID = 0;
    int apiID = cv::CAP_ANY;

    cameraAPI::CameraThread& acquisition_thread = cameraAPI::CameraThread::GetInstance();
    acquisition_thread.init(deviceID, apiID);
    acquisition_thread.start();

    std::vector<double> init_camera_position = {0,0,0,0,0,0};
    cameraPositions.init(init_camera_position);

    Reconstructor reconstructor;

    // perform calibration
    if (!reconstructor.init()) {
        return -1;
    }

    // TODO get it from command line parser
    int x_distance = 2; // meters
    double interval = 0.5; // meters

    std::string filepath = get_exec_path();
    std::vector<std::string> images_paths;

    int steps = static_cast<int>(static_cast<double>(x_distance)/interval);
    std::cout << "Going to move camera " << steps << " times by " << interval << " m.\n";

    for (int i = 0; i < steps; i++) {
        std::cout << "Step " << i+1 << std::endl;
        cameraPositions.move(interval);
        cv::Mat frame = acquisition_thread.read();

        if (frame.empty()) {
            std::cerr << "ERROR! Blank frame grabbed\n";
            break;
        }
        imshow("Grabbed frame", frame);

        std::string filename = filepath + "_reconstruction_" + std::to_string(i) + ".jpg";
        if (!cv::imwrite(filename, frame)) {
            std::cerr << "ERROR! Couldnt save a file: " << filename << "\n";
            break;
        }

        images_paths.push_back(filename);
        if (cv::waitKey(500) >= 0)
            break;
    }

    acquisition_thread.stop();

//    std::vector<cv::Vec3d> points3D = reconstructor.reconstruct(images_paths, cameraPositions);

//    // visualize
//    cv::viz::Viz3d window; //creating a Viz window

//    //Displaying the Coordinate Origin (0,0,0)
//    window.showWidget("coordinate", cv::viz::WCoordinateSystem());

//    window.setBackgroundColor(cv::viz::Color::black());

//    //Displaying the 3D points in green
//    window.showWidget("points", cv::viz::WCloud(points3D, cv::viz::Color::green()));
//    window.spin();

}
