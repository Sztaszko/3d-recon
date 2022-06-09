#include "utils.h"

void utils::help()
{
  std::cout
      << "\n------------------------------------------------------------------------------------\n"
      << " This program performs multiview incremental sfm reconstruction \n"
      << " with known camera positions while taking pictures \n"
      << " Usage:\n"
      << "        3d-recon <cameraParamsFile> <reconstructionFiles>\n"
      << " where: cameraParamsFile - is the file absolute path into your system which contains\n"
      << "        camera parameters obtained in calibration. Should be xml or yml. \n"
      << "        reconstructionFiles - is the file absolute path into your system which contains\n"
      << "        the list of images to use for reconstruction.\n"
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
