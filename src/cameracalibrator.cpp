#include "cameracalibrator.h"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include "opencv2/xfeatures2d.hpp"
#include <unistd.h>


// Open chessboard images and extract corner points
std::vector<std::string> CameraCalibrator::getCalibImages()
{
    _chessboard_files.clear();

    // Grab and write loop of calibration chessboard

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
        cv::Mat frame = cameraAPI::CameraThread::GetInstance().read();
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
        _chessboard_files.push_back(filename);

        if (cv::waitKey(250) >= 0)
            break;
    }


    return _chessboard_files;
}

int CameraCalibrator::addChessboardPoints(const std::vector<std::string>& filelist,
                                          cv::Size & boardSize)
{

    _board_size = boardSize;

    // the points on the chessboard
    std::vector<cv::Point2f> imageCorners;
    std::vector<cv::Point3f> objectCorners;

    float real_life_width = 0.023f; // chessboard field width in meters

    // 3D Scene Points:
    // Initialize the chessboard corners
    // in the chessboard reference frame
    // The corners are at 3D location (X,Y,Z)= (i,j,0)
    for (int i=0; i<_board_size.height; i++) {
        for (int j=0; j<_board_size.width; j++) {

          objectCorners.push_back(cv::Point3f(i*real_life_width, j*real_life_width, 0.0f));
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

//void CameraCalibrator::saveCameraParams(Settings& s, cv::Size& imageSize,const std::vector<cv::Point3f>& newObjPoints)
//{
//    cv::FileStorage fs( s.outputFileName, cv::FileStorage::WRITE );

//    time_t tm;
//    time( &tm );
//    struct tm *t2 = localtime( &tm );
//    char buf[1024];
//    strftime( buf, sizeof(buf), "%c", t2 );

//    fs << "calibration_time" << buf;

////    if( !rvecs.empty() || !reprojErrs.empty() )
////        fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
//    fs << "image_width" << imageSize.width;
//    fs << "image_height" << imageSize.height;
//    fs << "board_width" << s.boardSize.width;
//    fs << "board_height" << s.boardSize.height;
//    fs << "square_size" << s.squareSize;

//    if( !s.useFisheye && s.flag & cv::CALIB_FIX_ASPECT_RATIO )
//        fs << "fix_aspect_ratio" << s.aspectRatio;

//    if (s.flag)
//    {
//        std::stringstream flagsStringStream;
//        if (s.useFisheye)
//        {
//            flagsStringStream << "flags:"
//                << (s.flag & cv::fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
//                << (s.flag & cv::fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
//                << (s.flag & cv::fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
//                << (s.flag & cv::fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
//                << (s.flag & cv::fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
//                << (s.flag & cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
//        }
//        else
//        {
//            flagsStringStream << "flags:"
//                << (s.flag & cv::CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
//                << (s.flag & cv::CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
//                << (s.flag & cv::CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
//                << (s.flag & cv::CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
//                << (s.flag & cv::CALIB_FIX_K1 ? " +fix_k1" : "")
//                << (s.flag & cv::CALIB_FIX_K2 ? " +fix_k2" : "")
//                << (s.flag & cv::CALIB_FIX_K3 ? " +fix_k3" : "")
//                << (s.flag & cv::CALIB_FIX_K4 ? " +fix_k4" : "")
//                << (s.flag & cv::CALIB_FIX_K5 ? " +fix_k5" : "");
//        }
//        fs.writeComment(flagsStringStream.str());
//    }

//    fs << "flags" << s.flag;

//    fs << "fisheye_model" << s.useFisheye;

//    fs << "camera_matrix" << cameraMatrix;
//    fs << "distortion_coefficients" << distCoeffs;

////    fs << "avg_reprojection_error" << totalAvgErr;
////    if (s.writeExtrinsics && !reprojErrs.empty())
////        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

//    if(s.writeExtrinsics && !rvecs.empty() && !tvecs.empty() )
//    {
//        CV_Assert(rvecs[0].type() == tvecs[0].type());
//        cv::Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
//        bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
//        bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

//        for( size_t i = 0; i < rvecs.size(); i++ )
//        {
//            cv::Mat r = bigmat(cv::Range(int(i), int(i+1)), cv::Range(0,3));
//            cv::Mat t = bigmat(cv::Range(int(i), int(i+1)), cv::Range(3,6));

//            if(needReshapeR)
//                rvecs[i].reshape(1, 1).copyTo(r);
//            else
//            {
//                //*.t() is MatExpr (not Mat) so we can use assignment operator
//                CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
//                r = rvecs[i].t();
//            }

//            if(needReshapeT)
//                tvecs[i].reshape(1, 1).copyTo(t);
//            else
//            {
//                CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
//                t = tvecs[i].t();
//            }
//        }
//        fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
//        fs << "extrinsic_parameters" << bigmat;
//    }

//    if(s.writePoints && !imagePoints.empty() )
//    {
//        cv::Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
//        for( size_t i = 0; i < imagePoints.size(); i++ )
//        {
//            cv::Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
//            cv::Mat imgpti(imagePoints[i]);
//            imgpti.copyTo(r);
//        }
//        fs << "image_points" << imagePtMat;
//    }

//    if( s.writeGrid && !newObjPoints.empty() )
//    {
//        fs << "grid_points" << newObjPoints;
//    }
//}

