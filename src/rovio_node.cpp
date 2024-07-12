// /*
// * Copyright (c) 2014, Autonomous Systems Lab
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// * * Redistributions of source code must retain the above copyright
// * notice, this list of conditions and the following disclaimer.
// * * Redistributions in binary form must reproduce the above copyright
// * notice, this list of conditions and the following disclaimer in the
// * documentation and/or other materials provided with the distribution.
// * * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
// * names of its contributors may be used to endorse or promote products
// * derived from this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// */


// #include <memory>

// #include <Eigen/StdVector>

// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wunused-parameter"
// // #include <ros/ros.h>
// // #include <ros/package.h>
// // #include <geometry_msgs/Pose.h>
// #pragma GCC diagnostic pop

// #ifdef MAKE_SCENE
// #include "rovio/RovioScene.hpp"
// #endif

#ifdef ROVIO_NMAXFEATURE
static constexpr int nMax_ = ROVIO_NMAXFEATURE;
#else
static constexpr int nMax_ = 25; // Maximal number of considered features in the filter state.
#endif

#ifdef ROVIO_NLEVELS
static constexpr int nLevels_ = ROVIO_NLEVELS;
#else
static constexpr int nLevels_ = 4; // // Total number of pyramid levels considered.
#endif

#ifdef ROVIO_PATCHSIZE
static constexpr int patchSize_ = ROVIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 6; // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef ROVIO_NCAM
static constexpr int nCam_ = ROVIO_NCAM;
#else
static constexpr int nCam_ = 1; // Used total number of cameras.
#endif

#ifdef ROVIO_NPOSE
static constexpr int nPose_ = ROVIO_NPOSE;
#else
static constexpr int nPose_ = 0; // Additional pose states.
#endif


// #ifdef MAKE_SCENE
// static rovio::RovioScene<mtFilter> mRovioScene;

// void idleFunc(){
//   ros::spinOnce();
//   mRovioScene.drawScene(mRovioScene.mpFilter_->safe_);
// }
// #endif

#include "rovio/RovioFilter.hpp"
#include "rovio/RovioNode.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <string>
#include<dirent.h>
#include <filesystem>

#include <chrono>
#include <thread>
#define with_visual


typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;



int main(int argc, char** argv){


    // Filter
    std::shared_ptr<mtFilter> mpFilter(new mtFilter);
    std::string rootdir = ".";
    std::string filter_config = rootdir + "/cfg/rovio.info";
    mpFilter->readFromInfo(filter_config);

    // Load camera calibration files
    std::string camera_config = rootdir + "/cfg/euroc_cam0.yaml";
    mpFilter->cameraCalibrationFile_[0] = camera_config;
    mpFilter->refreshProperties();



    rovio::RovioNode<mtFilter> node(mpFilter);

// Hard coded IMU and image loading stuff that is only for this example
// Usually you would get the data from your camera and IMU directly (skip to 198)
//------------------------------------------------------------------------------------------------------------------//
    // Load IMU data, this should at some point come from IMU/FC directly instead of reading a csv
    std::ifstream file(rootdir + "/example/Vicon_Room_101/mav0/imu0/data.csv");

    // Check if the file was opened successfully
    if (!file.is_open()) {
        std::cout << "Failed to open IMU file" << std::endl;
        return 1;
    }

    // Read the data from the file
    std::string line;
    int count = 0;
    double imuStmp;
    double ax;
    double ay;
    double az;
    double wx;
    double wy;
    double wz;

    std::string::size_type sz;



    // create hardcoded list of camera time stamps that correspond to the image file name
    std::string img_file;
    std::vector<long int> camStmps;

    // https://stackoverflow.com/questions/11140483/how-to-get-list-of-files-with-a-specific-extension-in-a-given-folder/47975458#47975458
    std::string path(rootdir + "/example/Vicon_Room_101/left/");
    std::string ext(".png");
    for (auto &p : std::filesystem::recursive_directory_iterator(path))
    {
        if (p.path().extension() == ext)
        {
            camStmps.push_back(std::stold(p.path().stem().string()));
            //std::cout << p.path().stem().string() << '\n';
        }
    }

    // std::vector<long int> camStmps;
    std::sort(camStmps.begin(), camStmps.end());

    // counting the number of processed images and imu msgs
    int img_procd = 0;
    int count_imus = 0;

    std::getline(file, line);
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        while (std::getline(ss, line, ',')) {
            // load imu msg
            if (count%7 == 0)
              imuStmp = std::stod(line, &sz);
            else if (count%7 == 1)
              wx = std::stod(line, &sz);
            else if (count%7 == 2)
              wy = std::stod(line, &sz);
            else if (count%7 == 3)
              wz = std::stod(line, &sz);
            else if (count%7 == 4)
              ax = std::stod(line, &sz);
            else if (count%7 == 5)
              ay = std::stod(line, &sz);
            else if (count%7 == 6)
            {
              az = std::stod(line, &sz);
              count_imus++;
//----------------------------------------------------------------------------------------------------------------------------------//
              Eigen::Vector3d acc;
              Eigen::Vector3d gyro;
              acc << ax , ay, az;
              gyro << wx , wy, wz;

              // each time we receive/load a single IMU msg we run the imuCallback, which will put the message in a buffer
              // once the camera and IMU at the same time stamp are available it will do the prediction and update step
              node.imuCallback(acc, gyro, imuStmp*1e-9);
            }

            // here we load the next image if the IMU time stamp > than the image, but this is not a requirement
            // you can call the imgcallback right away when you receive an image.
            if (imuStmp > camStmps[img_procd])
            {
              // read image, so also this is not needed
//----------------------------------------------------------------------------------------------------------------------------------//
              img_file = std::to_string(camStmps[img_procd]);
              cv::Mat img = cv::imread(rootdir + "/example/Vicon_Room_101/left/" + img_file + ".png", cv::IMREAD_GRAYSCALE);
              double msg_time = camStmps[img_procd] *1e-9;
//----------------------------------------------------------------------------------------------------------------------------------//

              node.imgCallback(img, msg_time);

              // sleep  is again not need but is used to see the visualized frames @ 1000/50 = 20Hz
              std::this_thread::sleep_for(std::chrono::milliseconds(50));
              img_procd++;
            }

          count++;
        }
    }

    // Close the file
    file.close();



    while (true)
        continue;

    return 0;
}
