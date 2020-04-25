# rosbag2pcd2bin_converter 

This repo is made to make it easy for people to convert pointcloud data from rosbags to .bin files which has the same format as KITTI 3D object detection dataset

## Prerequisites

- ROS Melodic
- Python3
- PCL library 



## Usage
### Step 1:

convert your rosbag to pcd files by specifying the the path and topic but make sure to place the pcd files inside the pcd_files directory
```bash
$ mkdir pcd_files
$ rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <your_path/rosbag2pcd2bin_converter/pcd_files>
```

example:
```bash
 $ rosrun pcl_ros bag_to_pcd data.bag /laser_tilt_cloud path/rosbag2pcd2bin_converter/pcd_files
```

### Step 2:

Build and compile the c++ code by doing those commands
```bash
$ mkdir build
$ mkdir bin_files
$ cd build
$ cmake ..
$ make
```

### Step 3:

Run the code, from inside the build directory
```bash
$ ./pcd2bin_converter
```

### Step 4:

Rename .bin files to be the same as KITTI .bin files (ex: 000001.bin)

```bash
$ python3 rename.py
```



## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.



