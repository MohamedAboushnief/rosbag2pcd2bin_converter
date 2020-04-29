#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <string>

#include <dirent.h>
#include <unistd.h>
#include <numeric>
#include <sys/stat.h>
#include <sys/types.h>
#include <experimental/filesystem>

using namespace std;


void readKittiPclBinData(const std::experimental::filesystem::v1::__cxx11::directory_entry& entry, std::string& out_file)
{
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

    std::cout << "Read " <<fileName.c_str()<< " , with " << i << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;

    // Save DoN features
    writer.write< pcl::PointXYZI > (out_file, *points, false);
}

void readKittiPclBinData(std::string fileName, std::string& out_file)
{
    // load point cloud
    std::fstream input(fileName.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << fileName << std::endl;
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

    std::cout << "Read " <<fileName.c_str()<< " , with " << i << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;

    // Save DoN features
    writer.write< pcl::PointXYZI > (out_file, *points, false);
}

int main (int argc, char** argv){
    
  namespace fs = std::experimental::filesystem;
  std::string path = "../ouster_pcd_files";
  //  std::string fName ="test_bin.bin"; 
  for (const auto & entry : fs::directory_iterator(path)){
    std::cout << entry.path().filename() << std::endl;
    // auto fileName =  entry.path().filename();
    string rawname = entry.path().stem();
    string saveDataPathFile = "../ouster_bin_files/"+rawname+".bin";
    // string saveDataPathFile = fName;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> 
    (entry.path(), *cloud) == -1){
    PCL_ERROR ("Couldn't read file \n");
    return (-1);
    }
    int numOfPoints = cloud->points.size();
    int n =numOfPoints*4;
    float* data =  new float[n];  // Allocate n points and save ptr in data.
    // if you like you can init all elements to zero    
    for (int i=0; i<n; i++) {
        data[i] = 0;    // Initialize all elements to zero.
    }
    // check for max intensity value
    double maxIntensityPcd = 0;
    std::vector<float>intValues;
    for (std::size_t i = 0; i < cloud->points.size (); ++i){
        intValues.push_back(cloud->points[i].intensity);
    }
    float meanOfIntensity = 0.0f;
    if(numOfPoints!=0){
        meanOfIntensity = accumulate( intValues.begin(), intValues.end(), 0.0) / numOfPoints; 
    }
    
    // std::cout<<"Mean Intensity Value in PCD: "<< meanOfIntensity<<std::endl;
    float maxIntensity = 200; // Approx the max value of the velodyne 
    FILE * stream; 
    stream = fopen (saveDataPathFile.c_str() ,"wb");
    int errorCounter=0;
    float *it = data; // set pointer to data 
    for (std::size_t i = 0; i < cloud->points.size (); ++i){
        float intensityApprox = cloud->points[i].intensity/meanOfIntensity*maxIntensity;
        intensityApprox= intensityApprox > maxIntensity ? maxIntensity:intensityApprox;
        if ((intensityApprox <=0)or (isnan(cloud->points[i].x ))){
            errorCounter++;
            continue;}

        *(it++) = cloud->points[i].x;
        *(it++) = cloud->points[i].y;
        *(it++) = cloud->points[i].z;
            
        *(it++) =intensityApprox ;
    }
    fwrite(data,sizeof(float),n,stream);
    fclose(stream);
    std::cout<<"Skipped "<<errorCounter<<" points "<<std::endl;
    delete [] data;  // When done, free memory pointed to by a.
    data = NULL;     // Clear a to prevent using invalid memory reference.

  }

  /* Test stuff */
  
  // std::string out_file = "test_1.pcd";
  // readKittiPclBinData(fName, out_file);
  
  // test read one binary data 
//   std::string binPath = "../../bin_files";
  
//   for (const auto & entry : fs::directory_iterator(binPath)){
//     std::string out_file = "test_1.pcd";
//     readKittiPclBinData(entry, out_file);
//     break;
//   }
  // // test read binary data from kitty 
  // binPath = "../bin_kitty";
  
  // for (const auto & entry : fs::directory_iterator(binPath)){
  //   std::string out_file = "test_2.pcd";
  //   readKittiPclBinData(entry, out_file);
  // }

  return (0);
}