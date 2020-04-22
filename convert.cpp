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

using namespace std;

void readKittiPclBinData(const std::experimental::filesystem::v1::__cxx11::directory_entry& entry, std::string& out_file)
{//this function is for checking if the conversion was right or not
    std::cout <<"Read :"<< entry.path().filename() << std::endl;
    auto fileName =  entry.path().filename();
    // load point cloud
    std::fstream input(entry.path().c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << entry.path().c_str() << std::endl;
        return ;
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points->push_back(point);
    }
    input.close();
//    g_cloud_pub.publish( points );

    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;

    // Save DoN features
    writer.write< pcl::PointXYZI > (out_file, *points, false);
}

int main (int argc, char** argv){
    
  namespace fs = std::experimental::filesystem;
  std::string path = "../ouster_pcd_files";
  
  for (const auto & entry : fs::directory_iterator(path)){
    std::cout << entry.path().filename() << std::endl;
    // auto fileName =  entry.path().filename();
    string rawname = entry.path().stem();
    string saveDataPathFile = "../ouster_bin_files/"+rawname+".bin";
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
    
    // std::stringstream ss;
    // ss << "../bin_files" << "/" << entry.path().filename() << ".bin";

    FILE * stream; 
    // stream = fopen (ss.str().c_str() ,"wb");
    stream = fopen (saveDataPathFile.c_str() ,"wb");

    for (std::size_t i = 0; i < cloud->points.size (); ++i){
    // std::cout << "    " << cloud->points[i].x
    //           << " "    << cloud->points[i].y
    //           << " "    << cloud->points[i].z 
    //           << " "    << cloud->points[i].intensity << std::endl;

    float data[4]={cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,cloud->points[i].intensity};
    fwrite(data,sizeof(float),sizeof(data),stream);


  }

    fclose(stream);
    //break;

  }


  // test read binary data 
  // std::string binPath = "../velodyne_bin_files";
  
  // for (const auto & entry : fs::directory_iterator(binPath)){
  //   std::string out_file = "test_1.pcd";
  //   readKittiPclBinData(entry, out_file);
  //   break;
  // }
  // // test read binary data from kitty 
  // binPath = "../ouster_bin_files";
  
  // for (const auto & entry : fs::directory_iterator(binPath)){
  //   std::string out_file = "test_2.pcd";
  //   readKittiPclBinData(entry, out_file);
  // }

  // // test read binary data from kitty 
  // binPath = "../kitti_bin_files";
  
  // for (const auto & entry : fs::directory_iterator(binPath)){
  //   std::string out_file = "test_3.pcd";
  //   readKittiPclBinData(entry, out_file);
  // }


  return (0);
}
