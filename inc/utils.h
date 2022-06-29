#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <unistd.h>

namespace utils {

void help();

std::string get_exec_path();

std::vector<std::string> read_file_list(std::string filename);

//static int getdir(const std::string _filename, std::vector<cv::String> &files)
//{
//  std::ifstream myfile(_filename.c_str());
//  if (!myfile.is_open()) {
//    std::cout << "Unable to read file: " << _filename << std::endl;
//    exit(0);
//  } else {;
//    size_t found = _filename.find_last_of("/\\");
//    std::string line_str, path_to_file = _filename.substr(0, found);
//    while ( getline(myfile, line_str) )
//      files.push_back(path_to_file + std::string("/") + line_str);
//  }
//  return 1;
//}

bool save_point_cloud(std::string filename, std::vector<cv::Vec3d> pointCloud);

/*!
 * \brief The SandBox class - this is sandbox class used to test parts of the main program
 * it should not be considered as functional element of the program, neither neccessary
 * functionalities should be implemented here. To access for example private members of the class
 * the friend mechanism can be used
 */

}

#endif // UTILS_H
