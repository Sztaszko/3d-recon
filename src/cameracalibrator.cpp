#include "cameracalibrator.h"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include "opencv2/xfeatures2d.hpp"
#include <unistd.h>


// Open chessboard images and extract corner points
std::vector<std::string> CameraCalibrator::getCalibImages()
{
    std::vector<std::string> chessboard_files;

    // Grab and write loop of calibration chessboard

    cv::Mat frame;

    // get executing path
    char pBuf[256];
    size_t len = sizeof(pBuf);

    int bytes = MIN(readlink("/proc/self/exe", pBuf, len), len - 1);
    if(bytes >= 0)
        pBuf[bytes] = '\0';

    std::string filepath(pBuf);

    for (int i=0; i<_images_count; ++i)
    {
        // wait for a new frame from camera and store it into 'frame'
        _camera->read(frame);
        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
        imshow("Live", frame);

        std::string filename = filepath + std::to_string(i) + ".jpg";
        if (!cv::imwrite(filename, frame)) {
            std::cerr << "ERROR! Couldnt save a file: " << filename << "\n";
            break;
        }
        chessboard_files.push_back(filename);

        if (cv::waitKey(50) >= 0)
            break;
    }

    return chessboard_files;
}

int CameraCalibrator::addChessboardPoints(const std::vector<std::string>& filelist,
                                          cv::Size & boardSize)
{

    _board_size = boardSize;

    // the points on the chessboard
    std::vector<cv::Point2f> imageCorners;
    std::vector<cv::Point3f> objectCorners;


    // 3D Scene Points:
    // Initialize the chessboard corners
    // in the chessboard reference frame
    // The corners are at 3D location (X,Y,Z)= (i,j,0)
    for (int i=0; i<_board_size.height; i++) {
        for (int j=0; j<_board_size.width; j++) {

          objectCorners.push_back(cv::Point3f(i, j, 0.0f));
        }
    }

    // 2D Image points:
    cv::Mat image; // to contain chessboard image
    int successes = 0;
    // for all viewpoints
    for (int i=0; i<filelist.size(); i++) {

        std::cout << "Read file: " << filelist[i] << "\n";

        // Open the image
        image = cv::imread(filelist[i], 0);

        // Get the chessboard corners
        bool found = cv::findChessboardCorners(image, _board_size, imageCorners, CV_CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

        if (!found || imageCorners.empty()) {
            std::cout << "ERROR! Not all chessboard corners found. Only " << imageCorners.size() << " were found.\n";
            continue;
        }

        // Get subpixel accuracy on the corners
        cv::cornerSubPix(image, imageCorners,
                          cv::Size(5,5),
                          cv::Size(-1,-1),
                          cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                          cv::TermCriteria::EPS,
                          30,    // max number of iterations
                          0.1));     // min accuracy

      // If we have a good board, add it to our data
      if (imageCorners.size() == _board_size.area()) {

            // Add image and scene points from one view
            addPoints(imageCorners, objectCorners);
            successes++;
          }



        //Draw the corners
        cv::drawChessboardCorners(image, _board_size, imageCorners, found);
        cv::imshow("Corners on Chessboard", image);
        cv::waitKey(100);
    }

    return successes;
}

// Add scene points and corresponding image points
void CameraCalibrator::addPoints(const std::vector<cv::Point2f>& imageCorners,
                                 const std::vector<cv::Point3f>& objectCorners)
{

  // 2D image points from one view
  imagePoints.push_back(imageCorners);
  // corresponding 3D scene points
  objectPoints.push_back(objectCorners);

}

// Calibrate the camera
// returns the re-projection error
double CameraCalibrator::calibrate(cv::Size &imageSize)
{
  // undistorter must be reinitialized

  //Output rotations and translations

  // start calibration
  reprojection_err = calibrateCamera(objectPoints, // the 3D points
                                     imagePoints,  // the image points
                                     imageSize,    // image size
                                     cameraMatrix, // output camera matrix
                                     distCoeffs,   // output distortion matrix
                                     rvecs, tvecs, // Rs, Ts
                                     flag);        // set options
                           //          ,CV_CALIB_USE_INTRINSIC_GUESS);

//vector<Point3f> newObjPoints;
//  reprojection_err = calibrateCameraRO(objectPoints, // the 3D points
//                                     imagePoints,  // the image points
//                                     imageSize,    // image size
//                                     imageSize.width - 1,
//                                     cameraMatrix, // output camera matrix
//                                     distCoeffs,   // output distortion matrix
//                                     rvecs, tvecs, // Rs, Ts
//                                     newObjPoints,
//                                     flag);        // set options


  return reprojection_err;
}
