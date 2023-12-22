# README #

### Building
* git checkout fast_bare_bones
* git submodule update --init
* mkdir build
* cd build
* cmake ..
* make -j4


### How to use it for your camera
in rovio_node.cpp the node is created (rovio::RovioNode<mtFilter> node(mpFilter);)  
then with each new frame or imu message you should call the callbacks: node.imgCallback(img, msg_time);and node.imuCallback(acc, gyro, imuStmp*1e-9);  
for now it coded to load the images in the example folder with the imu messages  
  
The settings for ROVIO can be adjusted in cfg/rovio.info  
    std::string filter_config = rootdir + "/cfg/rovio.info";  
    mpFilter->readFromInfo(filter_config);  
The camera callibartion should be done with Kalibr (https://github.com/ethz-asl/kalibr) or at least be in the same format. (I am not sure, but I thought the old docker works, ubuntu 16 or 18, while the newer one does not)  
Then the output of the callbiaration (yaml file) should be put in cfg folder and the camera_config name should be changed in rovio_node.cpp  
    // Load camera calibration files  
    std::string camera_config = rootdir + "/cfg/euroc_cam0.yaml"; // change this here to your .yaml  
    mpFilter->cameraCalibrationFile_[0] = camera_config;  
    mpFilter->refreshProperties();  
  



### Modifications to ROVIO in this fork ###
The main changes is that we use sparse matrix multiplications in the state prediction and update step of the IEKF. The sparsity of the prediction step could be derived from the EOMs. For the update step, the Jacobian is essentially a 2x2 matrix, (QR decompositon of image gradient and light condition params), using this 2x2 matrix instead of (2x21+3*max_features) allows to reduce the computational cost of many steps in the update step of the IEKF. Furthermore, we did some clean up of unnecessary computation (for example, calculating the same image gradient at every iteration, instead of just once per feature, per frame). This resulted in a computational cost reduction of 40%, using the original settings on an Nvidia Jetson TX2. 
These modifications has been presented at IMAV 2022:
"Improving the computational efficiency of ROVIO"
by S.A. Bahnam, C. de Wagter, and G.C.H.E de Croon
from Delft University of Technology, Kluyverweg 1, Delft

Furthermore, we use the FAST score instead of the Shi-Tomasi score for feature selection. We only detect features on 1/4 image size (instead 1/2 and 1/4). When many features candidates are detected (more than 250) the 150 features with the highest FAST score are pre-selected. With those modifications we were able to run ROVIO on a RPI Zero 2 W with the original number of features (25). However, when reducing the number of features to 15 it is more stable as it processes more frames. It is important to reduce the camera buffer (to 1), so it does not run out of the 512 MB RAM. We also recommend to disable SWAP memory (with sudo dphys-swapfile swapoff).

### New Parameters and Defintions ###
To check the difference between original ROVIO and sparse ROVIO with Eigen::isApprox()
* #define CHECK_GENERATE_MATRICES in ImgUpdate.hpp
* #define CHECK_COV_PREDICTION_MATRICES in Prediction.hpp
* #define CHECK_UPDATE_MATRICES in Update.hpp


For computation time logging (do not forget to change directory name)
* #define IMG_CALLBACK_LOG in RovioNode.hpp
* #define TIME_LOG in RovioNode.hpp
* #define UPDATELOG in Update.hpp
* #define UPDATELOG_PER_FRAME in Update.hpp

To save the VIO estimate in a text file (do not forget to change directory name)
* #define SAVE_VIO_TO_TXT in RovioNoe.hpp

If you change the number of features, currently you have to modify Prediction.hpp because the sparse matrix multiplication is hard coded. We will update late, so you won't have to worry about it. (You can change the number features in CMakeLists.txt)

For EuRoC:
* in RovioNode.hpp subrscribe to cam0/image_raw and imu0
* set the camera buffer and IMU buffer to your preference. Make the IMU buffer at least 200 times bigger than the camera buffer (else your prediction step will be incorrect).
* use roslaunch rovio  rovio_node.launch

For the UZH-FPV drone racing dataset use:
* in RovioNode.hpp subrscribe to /snappy_cam/stereo_l and /snappy_imu
* use roslaunch rovio  rovio_node_uzhfpv.launch


### ROVIO ###

This repository contains the ROVIO (Robust Visual Inertial Odometry) framework. The code is open-source (BSD License). Please remember that it is strongly coupled to on-going research and thus some parts are not fully mature yet. Furthermore, the code will also be subject to changes in the future which could include greater re-factoring of some parts.

Video: https://youtu.be/ZMAISVy-6ao

Papers:
* http://dx.doi.org/10.3929/ethz-a-010566547 (IROS 2015)
* http://dx.doi.org/10.1177/0278364917728574 (IJRR 2017)

Please also have a look at the wiki: https://github.com/ethz-asl/rovio/wiki

### Install without opengl scene ###
Dependencies:
* ros
* kindr (https://github.com/ethz-asl/kindr)
* lightweight_filtering (as submodule, use "git submodule update --init --recursive")

```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Install with opengl scene ###
Additional dependencies: opengl, glut, glew (sudo apt-get install freeglut3-dev, sudo apt-get install libglew-dev)
```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```

### Euroc Datasets ###
The rovio_node.launch file loads parameters such that ROVIO runs properly on the Euroc datasets. The datasets are available under:
http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

### Further notes ###
* Camera matrix and distortion parameters should be provided by a yaml file or loaded through rosparam
* The cfg/rovio.info provides most parameters for rovio. The camera extrinsics qCM (quaternion from IMU to camera frame, Hamilton-convention) and MrMC (Translation between IMU and Camera expressed in the IMU frame) should also be set there. They are being estimated during runtime so only a rough guess should be sufficient.
* Especially for application with little motion fixing the IMU-camera extrinsics can be beneficial. This can be done by setting the parameter doVECalibration to false. Please be carefull that the overall robustness and accuracy can be very sensitive to bad extrinsic calibrations.
