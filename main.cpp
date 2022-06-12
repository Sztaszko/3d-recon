#define CERES_FOUND 1

#include <iostream>
#include <fstream>

#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/core.hpp>
#include "opencv2/xfeatures2d.hpp"

#include "reconstruction.h"
#include "cameracalibrator.h"
#include "camera.h"
#include "utils.h"


int main(int argc, char *argv[]){

    const cv::String keys =
                "{help h usage ?        |                        | print this message                       }"
                "{@cameraParams         |<none>                  | input camera file                        }"
                "{@reconstructionFiles  |<none>                  | input list of images and cam positions   }"
                "{d distance            |2                       | distance of handCamera trajectory <meters>}"
                "{i interval            |0.2                     | interval between pictures        <meters>}"
                "{c camera              |1                       | camera ID                                }"
            ;

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("This is structure from motion pipeline implementation with known position of the camera.\n");

    if (parser.has("help"))
    {
        parser.printMessage();
        utils::help();
        return 0;
    }

    std::string cameraParamsFile = parser.get<cv::String>("@cameraParams");
    std::string reconstructionFiles = parser.get<cv::String>("@reconstructionFiles");
    double x_distance = parser.get<double>("d");
    double interval = parser.get<double>("i");
    int deviceID = parser.get<int>("c");

    int apiID = cv::CAP_ANY;

    cameraAPI::HandCameraPosition cameraPositions;
    Reconstructor reconstructor;
    std::vector<std::string> images_paths;

    // create acquisition thread
    cameraAPI::CameraThread& acquisition_thread = cameraAPI::CameraThread::GetInstance();
    acquisition_thread.init(deviceID, apiID);
    acquisition_thread.start();

    if (!cameraParamsFile.empty()) {
        std::string extension = cameraParamsFile.substr(cameraParamsFile.find_last_of(".") + 1);

        if (extension != "xml" && extension != "yml") {
            std::cout << "Incorrect file extension.\n"
                      << "Please use xml or yml files.\n";
            return -1;
        }

        // load params from file
        if (!reconstructor.init(cameraParamsFile)) {
            return -1;
        }

    } else {
        // perform calibration
        if (!reconstructor.init()) {
            return -1;
        }
    }

    if (!reconstructionFiles.empty()) {
        cameraPositions.read(reconstructionFiles);
        images_paths = utils::read_file_list(reconstructionFiles);

    } else {
        //acquisition loop

        cameraPositions.clear();

        std::string filepath = utils::get_exec_path();

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

        std::vector<cv::Matx44f> camera_extrinsics = cameraPositions.get_camera_extrinsics();
        cv::FileStorage images_file("images.xml", cv::FileStorage::WRITE);
        images_file << "filenames" << images_paths;
        images_file << "steps" << (int) camera_extrinsics.size();
        for (int j = 0; j < camera_extrinsics.size(); ++j) {
            images_file << "position" + std::to_string(j) << camera_extrinsics[j];
        }
        images_file.release();
    }


    acquisition_thread.stop();

    std::vector<cv::Vec3d> points3D = reconstructor.reconstruct(images_paths, cameraPositions);

//    // visualize
//    cv::viz::Viz3d window; //creating a Viz window

//    //Displaying the Coordinate Origin (0,0,0)
//    window.showWidget("coordinate", cv::viz::WCoordinateSystem());

//    window.setBackgroundColor(cv::viz::Color::black());

//    //Displaying the 3D points in green
//    window.showWidget("points", cv::viz::WCloud(points3D, cv::viz::Color::green()));
//    window.spin();

}
