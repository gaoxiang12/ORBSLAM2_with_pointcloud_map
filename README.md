# ORBSLAM2_with_pointcloud_map
This is a modified ORB_SLAM2 (from https://github.com/raulmur/ORB_SLAM2, thanks for Raul's great work!) with a online point cloud map module running in RGB-D mode. You can visualize your point cloud map during the SLAM process. 

# How to Install
Unzip the file you will find two directories. First compile the modified g2o:

```
  cd g2o_with_orbslam2
  mkdir build
  cd build
  cmake ..
  make 
```

Following the instructions from the original g2o library: [https://github.com/RainerKuemmerle/g2o] if you have dependency problems. I just add the extra vertecies and edges provided in ORB_SLAM2 into g2o. 

Then compile the ORB_SLAM2. You need firstly to compile the DBoW2 in ORB_SLAM2_modified/Thirdpary, and then the Pangolin module (https://github.com/stevenlovegrove/Pangolin). Finally, build ORB_SLAM2:

```
cd ORB_SLAM2_modified
mkdir build
cd build
cmake ..
make
```

To run the program you also need to download the ORB vocabulary (which is a large file so I don't upload it) in the original ORB_SLAM2 repository.

# Run examples
Prepare a RGBD camera or dataset, give the correct parameters and you can get a ORB SLAM with point cloud maps like the example.jpg in this repo.

# Build the unpacked modified repo 

please see this [README](./ORB_SLAM2_modified/README.md)
