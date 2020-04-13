#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <string>

#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <experimental/filesystem>


int main (int argc, char** argv){
    
  namespace fs = std::experimental::filesystem;
  std::string path = "../pcd_files";

  for (const auto & entry : fs::directory_iterator(path)){
    std::cout << entry.path().filename() << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> 
    (entry.path(), *cloud) == -1){
    PCL_ERROR ("Couldn't read file \n");
    return (-1);
    }

    std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
    
    std::stringstream ss;
    ss << "../bin_files" << "/" << entry.path().filename() << ".bin";

    FILE * stream; 
    stream = fopen (ss.str().c_str() ,"wb");

    for (std::size_t i = 0; i < cloud->points.size (); ++i){
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z 
              << " "    << cloud->points[i].intensity << std::endl;

    float data[4]={cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,cloud->points[i].intensity};
    fwrite(data,sizeof(float),sizeof(data),stream);

  }
    fclose(stream);

  }




  return (0);
}