#include "utils.h"
#include <opencv2/core/persistence.hpp>

void utils::help()
{
  std::cout
      << "\n------------------------------------------------------------------------------------\n"
      << " This program performs multiview incremental sfm reconstruction \n"
      << " with known camera positions while taking pictures \n"
      << " Usage:\n"
      << "        3d-recon <cameraParamsFile> <reconstructionFiles> <distance> <interval>\n"
      << " where: cameraParamsFile - is the file absolute path into your system which contains\n"
      << "        camera parameters obtained in calibration. Should be xml or yml. \n"
      << "        reconstructionFiles - is the file absolute path into your system which contains\n"
      << "        the list of images to use for reconstruction.\n"
      << "        distance - distance of the hand camera movement\n"
      << "        interval - interval of the distance between pictures\n"
      << "------------------------------------------------------------------------------------\n\n"
      << std::endl;
}

std::string utils::get_exec_path()
{
    // get executing path
    char pBuf[256];
    size_t len = sizeof(pBuf);

    int bytes = MIN(readlink("/proc/self/exe", pBuf, len), len - 1);
    if(bytes >= 0)
        pBuf[bytes] = '\0';

    std::string path(pBuf);
    return path;
}

std::vector<std::string> utils::read_file_list(std::string filename)
{
    cv::FileStorage fs;

    if (!fs.open(filename, cv::FileStorage::READ)) {
        std::cout << "Couldnt open " << filename << std::endl;
        return std::vector<std::string>();
    }

    std::vector<std::string> ret_val;
    fs["filenames"] >> ret_val;
    return ret_val;
}
