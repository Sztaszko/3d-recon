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

bool utils::save_point_cloud(std::string filename, std::vector<cv::Vec3d> pointCloud)
{
    std::ofstream outfile(filename);
    outfile << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
    outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << pointCloud.size() << "\n";
    outfile << "property float x\n" << "property float y\n" << "property float z\n" << "element face 0\n";
    outfile << "property list uchar int vertex_indices\n" << "end_header\n";
    for (int i = 0; i < pointCloud.size(); i++)
    {
        cv::Vec3d point = pointCloud.at(i);
        outfile << point[0] << " ";
        outfile << point[1] << " ";
        outfile << point[2] << " ";
        outfile << "\n";
    }
    outfile.close();

    return true;
}
