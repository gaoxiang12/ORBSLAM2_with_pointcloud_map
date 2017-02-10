# Update for unpacked ROB_SLAM with pcl view repo

## build:

### build the whole project ( inclouding binary loading tools ):

Before all the cmd, **DONOT** forget to download the Vocabulary form the [origin repo](https://github.com/raulmur/ORB_SLAM2) and place it into dir ./Vocabulary

```bash
     cd YourDirectory/ORBSLAM2_with_pointcloud_map
     chmod +x build.sh
     ./build.sh
```

### only build the ORB_SLAM2 mode with pcl

```bash
    cd YourDirectory/ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified
    mkdir build
    cd build
    cmake ..
    make -j
```

## Run:

```bash
    ./run/rgbd_tum Vocabulary/ORBvoc.bin path_to_settings path_to_sequence path_to_association
```

# What are modified:

* changing the vocabulary loading method into binary mod

* adding a pointcloud viewer( realized by adding a viewer thread )

* changing the CMakeLists.txt that all the executable files are placed in ./bin
