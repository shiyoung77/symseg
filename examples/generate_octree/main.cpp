/*****************************************************************************/
/*  This source code is for octree generation given camera positions and corresponding point clouds */
/*****************************************************************************/

#include <iostream>
#include <thread>
#include <experimental/filesystem>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>

// Utilities includes
#include "eigen.hpp"

// octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// boost
#include <boost/algorithm/string.hpp>

using namespace octomap;
using namespace std;
using PointNC = pcl::PointXYZRGBNormal;
namespace fs = std::experimental::filesystem;

int main(int argc, char** argv)
{
  OcTree tree(0.005);

  fs::path dataset = "/home/lsy/software/ssl/dataset";
  fs::path video = "0051";
  fs::path video_folder = dataset / video;
  fs::path pcd_info_path = video_folder / "sampled_pcd" / "info.txt";

  ifstream file(pcd_info_path);
  if (!file.is_open()) {
    cerr << pcd_info_path << " doesn't exist." << endl;
  }

  string line;
  vector<fs::path> pcd_paths;
  vector<point3d> cam_origins;

  getline(file, line);  // ignore the first line.
  string pcd_path, x, y, z;
  while (!file.eof()) {
    file >> pcd_path >> x >> y >> z;
    pcd_paths.emplace_back(pcd_path);
    cam_origins.emplace_back(stof(x), stof(y), stof(z));
    cout << pcd_paths.back() << " " << cam_origins.back() << endl;
  }
  file.close();

  for (size_t i = 0; i < 1; ++i) {
    const fs::path pcd_path = pcd_paths[i];
    const point3d sensor_origin = cam_origins[i];

    fs::path pcd_full_path = video_folder / "sampled_pcd" / pcd_path;
    pcl::PointCloud<PointNC>::Ptr pcd(new pcl::PointCloud<PointNC>);
    pcl::io::loadPCDFile(pcd_full_path, *pcd);

    Pointcloud scan;
    for (const auto &point : pcd->points) {
      point3d pt(point.x, point.y, point.z);
      scan.push_back(pt);
    }
    tree.insertPointCloud(scan, sensor_origin);
    cout << "inserted scen from: " << pcd_path << endl;
  }

  tree.writeBinary("simple_tree.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  

  return 0;
}
