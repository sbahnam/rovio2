# README #

### Modifications to ROVIO in this fork ###
The first main change is that we use sparse matrix multiplications in the state prediction and update step of the IEKF. The sparsity of the prediction step could be derived from the EOMs. For the update step, the Jacobian is essentially a 2x2 matrix, (QR decompositon of image gradient and light condition params), using this 2x2 matrix instead of (2x21+3*max_features) allows to reduce the computational cost of many steps in the update step of the IEKF. Furthermore, we did some clean up of unnecessary computation. This resulted in a computational cost reduction of 40%, using the original settings on an Nvidia Jetson TX2. 
These modifications has been presented at IMAV 2022:
"Improving the computational efficiency of ROVIO"
by S.A. Bahnam, C. de Wagter, and G.C.H.E de Croon
from Delft University of Technology, Kluyverweg 1, Delft

Secondly, we use a cheaper feature selection method that reduces the computational peaks of the algorithm. Use git checkout sparse_rovio to get the code where we make ROVIO computationally more efficient without affecting the accuracy and use git checkout fast_rovio to get the code where we also use the cheap fature selection method.


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
