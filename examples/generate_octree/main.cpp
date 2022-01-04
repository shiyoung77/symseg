/*****************************************************************************/
/*  This source code is for octree generation given camera positions and corresponding point clouds */
/*****************************************************************************/

#include <iostream>
#include <thread>
#include <experimental/filesystem>

// PCL
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>

// Utilities includes
#include "eigen.hpp"

// octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// boost
#include <boost/algorithm/string.hpp>

using namespace std;
using PointNC = pcl::PointXYZRGBNormal;
using path = std::experimental::filesystem::path;

int main(int argc, char** argv)
{
  octomap::OcTree tree(0.005);

  string dataset_str;
  string video;
  pcl::console::parse(argc, argv, "-dataset", dataset_str);
  pcl::console::parse(argc, argv, "-video", video);

  path dataset(dataset_str);
  path video_folder = dataset / video;
  path pcd_info_path = video_folder / "sampled_pcd" / "info.txt";

  ifstream file(pcd_info_path);
  if (!file.is_open()) {
    cerr << pcd_info_path << " doesn't exist." << endl;
  }

  string line;
  vector<path> pcd_paths;
  vector<octomap::point3d> cam_origins;

  getline(file, line);  // ignore the first line.
  string pcd_path, x, y, z;
  while (!file.eof()) {
    file >> pcd_path >> x >> y >> z;
    pcd_paths.emplace_back(pcd_path);
    cam_origins.emplace_back(stof(x), stof(y), stof(z));
    cout << pcd_paths.back() << " " << cam_origins.back() << endl;
  }
  file.close();

  for (size_t i = 0; i < pcd_paths.size(); ++i) {
    const path pcd_path = pcd_paths[i];
    const octomap::point3d sensor_origin = cam_origins[i];

    path pcd_full_path = video_folder / "sampled_pcd" / pcd_path;
    cout << "inserted point cloud from: " << pcd_full_path << endl;

    pcl::PointCloud<PointNC>::Ptr pcd(new pcl::PointCloud<PointNC>);
    pcl::io::loadPCDFile(pcd_full_path, *pcd);

    octomap::Pointcloud scan;
    for (const auto &point : pcd->points) {
      octomap::point3d pt(point.x, point.y, point.z);
      scan.push_back(pt);
    }
    tree.insertPointCloud(scan, sensor_origin, /*max_range=*/-1, /*lazy_eval=*/true, /*discretize=*/false);
  }
  // https://octomap.github.io/octomap/doc/classoctomap_1_1OccupancyOcTreeBase.html#a3c6d38e8a7028416cd23449f14e215e8
  tree.updateInnerOccupancy();

  path output_path = video_folder / "sampled_pcd" / "occupancy.bt";
  tree.writeBinary(output_path);

  cout << "octree written to: " << output_path << endl;
  cout << "now you can use octovis to visualize: octovis " << output_path  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;

  return 0;
}
