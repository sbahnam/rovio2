/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_ROVIONODE_HPP_
#define ROVIO_ROVIONODE_HPP_
#define with_visual

#include <memory>
#include <mutex>
#include <queue>

#include <iostream>
#include <fstream>

//#include <cv_bridge/cv_bridge.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/TwistWithCovarianceStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/image_encodings.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <std_srvs/Empty.h>
// #include <tf/transform_broadcaster.h>
// #include <visualization_msgs/Marker.h>

// #include <rovio/SrvResetToPose.h>
#include <iomanip>
#include "rovio/RovioFilter.hpp"


// Those logs slow down the algorithm, so use the only for debugging!
// #define IMG_CALLBACK_LOG
// #define TIME_LOG
// #define SAVE_VIO_TO_TXT

namespace rovio {

/** \brief Class, defining the Rovio Node
 *
 *  @tparam FILTER  - \ref rovio::RovioFilter
 */
template<typename FILTER>
class RovioNode{
 public:
  // Filter Stuff
  typedef FILTER mtFilter;
  std::shared_ptr<mtFilter> mpFilter_;
  typedef typename mtFilter::mtFilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;
  typedef typename mtFilter::mtPrediction::mtMeas mtPredictionMeas;
  mtPredictionMeas predictionMeas_;
  typedef typename std::tuple_element<0,typename mtFilter::mtUpdates>::type mtImgUpdate;
  typedef typename mtImgUpdate::mtMeas mtImgMeas;
  mtImgMeas imgUpdateMeas_;
  mtImgUpdate* mpImgUpdate_;
  // typedef typename std::tuple_element<1,typename mtFilter::mtUpdates>::type mtPoseUpdate;
  // typedef typename mtPoseUpdate::mtMeas mtPoseMeas;
  // mtPoseMeas poseUpdateMeas_;
  // mtPoseUpdate* mpPoseUpdate_;

  struct FilterInitializationState {
    FilterInitializationState()
        : WrWM_(V3D::Zero()),
          state_(State::WaitForInitUsingAccel) {}

    enum class State {
      // Initialize the filter using accelerometer measurement on the next
      // opportunity.
      WaitForInitUsingAccel,
      // Initialize the filter using an external pose on the next opportunity.
      // WaitForInitExternalPose,
      // The filter is initialized.
      Initialized
    } state_;

    // Buffer to hold the initial pose that should be set during initialization
    // with the state WaitForInitExternalPose.
    V3D WrWM_;
    QPD qMW_;

    explicit operator bool() const {
      return isInitialized();
    }

    bool isInitialized() const {
      return (state_ == State::Initialized);
    }
  };
  FilterInitializationState init_state_;

  bool forceOdometryPublishing_;
  bool forcePoseWithCovariancePublishing_;
  bool forceTransformPublishing_;
  bool forceExtrinsicsPublishing_;
  bool forceImuBiasPublishing_;
  bool forcePclPublishing_;
  bool forceMarkersPublishing_;
  bool forcePatchPublishing_;
  bool gotFirstMessages_;
  std::mutex m_filter_;

  // Nodes, Subscriber, Publishers


  // Ros Messages


  // Rovio outputs and coordinate transformations
  // typedef StandardOutput mtOutput;

  // mtOutput imuOutput_;
  // ImuOutputCT<mtState> imuOutputCT_;

  // ROS names for output tf frames.
  std::string map_frame_;
  std::string world_frame_;
  std::string camera_frame_;
  std::string imu_frame_;

  /** \brief Constructor
   */
  RovioNode(std::shared_ptr<mtFilter> mpFilter)
      : mpFilter_(mpFilter){
    mpImgUpdate_ = &std::get<0>(mpFilter_->mUpdates_);
    // mpPoseUpdate_ = &std::get<1>(mpFilter_->mUpdates_);
    forceOdometryPublishing_ = false;
    forcePoseWithCovariancePublishing_ = false;
    forceTransformPublishing_ = false;
    forceExtrinsicsPublishing_ = false;
    forceImuBiasPublishing_ = false;
    forcePclPublishing_ = false;
    forceMarkersPublishing_ = false;
    forcePatchPublishing_ = false;
    gotFirstMessages_ = false;

  }

  /** \brief Destructor
   */
  virtual ~RovioNode(){}




  /** \brief Callback for IMU-Messages. Adds IMU measurements (as prediction measurements) to the filter.
   */
  void imuCallback( Eigen::Vector3d acc,  Eigen::Vector3d gyro, double msgTime){
    // lock the img callback while we are in the IMU callback
    std::lock_guard<std::mutex> lock(m_filter_);
    predictionMeas_.template get<mtPredictionMeas::_acc>() = acc;
    predictionMeas_.template get<mtPredictionMeas::_gyr>() = gyro;
    
    // if the filter is initialized we add the IMU to the buffer
    if(init_state_.isInitialized()){
      mpFilter_->addPredictionMeas(predictionMeas_,msgTime);
      updateAndPublish();
    }
    // If not initialized we use the accelerometer to initialize (find the gravity vector), assuming that we are not moving
    else {
      std::cout << "-- Filter: Initializing using accel. measurement ..." << std::endl;
      mpFilter_->resetWithAccelerometer(predictionMeas_.template get<mtPredictionMeas::_acc>(),msgTime);

      std::cout << std::setprecision(18);
      std::cout << "-- Filter: Initialized at t = " << msgTime << std::endl;
      init_state_.state_ = FilterInitializationState::State::Initialized;
    }
  }


  /** \brief Image callback. Adds images (as update measurements) to the filter.
   *
   *   @param img   - Image message.
   *   @param camID - Camera ID.
   */
  void imgCallback(cv::Mat & cv_img_r, double msgTime, const int camID = 0){
    // here we should convert the (raw) loaded image to cv format
    cv::Mat cv_img = cv_img_r.clone();

    if(init_state_.isInitialized() && !cv_img.empty()){
      if(msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_){
        if(imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[0]){
          std::cout << "    \033[31mFailed Synchronization of Camera Frames, t = " << msgTime << "\033[0m" << std::endl;
        }
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
      }
      //create image pyramid (different scales)
      imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[camID].computeFromImage(cv_img,true);
      imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[camID] = true;

      if(imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid()){
        mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_,msgTime);
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);

        //here we run the filter
        updateAndPublish();
      }
    }
  }


  /** \brief Executes the update step of the filter and publishes the updated data.
   */
  void updateAndPublish(){
    if(init_state_.isInitialized()){
      // Execute the filter update.
      #ifdef with_visual
        const double t1 = (double) cv::getTickCount();
        static double timing_T = 0;
        static int timing_C = 0;
      #endif
      const double oldSafeTime = mpFilter_->safe_.t_;
      int c1 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      double lastImageTime;
      if(std::get<0>(mpFilter_->updateTimelineTuple_).getLastTime(lastImageTime)){
        // here we run the filter (prediction + update) if the imu messages and camera are both available
        mpFilter_->updateSafe(&lastImageTime);
      }
      #ifdef with_visual
      const double t2 = (double) cv::getTickCount();
      int c2 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      timing_T += (t2-t1)/cv::getTickFrequency()*1000;
      timing_C += c1-c2;

      #ifdef TIME_LOG
        if (c2 != c1)
        {
          int features_count = 0; 
          for (unsigned int i=0;i<mtState::nMax_; i++) {
            if(mpFilter_->safe_.fsm_.isValid_[i]){
              features_count++;
            }
          }
          std::ofstream logfile;
          logfile.open ("/home/stavrow/fpv_dataset/results/time_log.txt", std::ios::app);
          logfile << features_count << " " << (t2-t1)/cv::getTickFrequency()*1000 <<std::endl;
          logfile.close();
        }
      #endif
      #endif

      if(mpFilter_->safe_.t_ > oldSafeTime){ // Publish only if something changed
        #ifdef with_visual
          for(int i=0;i<mtState::nCam_;i++){
            if(!mpFilter_->safe_.img_[i].empty() && mpImgUpdate_->doFrameVisualisation_){
              cv::imshow("Tracker" + std::to_string(i), mpFilter_->safe_.img_[i]);
              cv::waitKey(3);
            }
          }
          if(!mpFilter_->safe_.patchDrawing_.empty() && mpImgUpdate_->visualizePatches_){
            cv::imshow("Patches", mpFilter_->safe_.patchDrawing_);
            cv::waitKey(3);
          }
        #endif

        // Obtain the save filter state.
        mtFilterState& filterState = mpFilter_->safe_;
	      mtState& state = mpFilter_->safe_.state_;
        state.updateMultiCameraExtrinsics(&mpFilter_->multiCamera_);
        MXD& cov = mpFilter_->safe_.cov_;
        // imuOutputCT_.transformState(state,imuOutput_);


        #ifdef SAVE_VIO_TO_TXT
          std::ofstream myfile;
          myfile.open ("/home/stavrow/fpv_dataset/results/vio_estimate.txt", std::ios::app);
          myfile <<std::fixed<<mpFilter_->safe_.t_<< " " << imuOutput_.WrWB()(0) << " " << imuOutput_.WrWB()(1)<< " " << imuOutput_.WrWB()(2)<< " " << imuOutput_.qBW().x()<< " " <<imuOutput_.qBW().y()<< " " <<imuOutput_.qBW().z()<< " " <<-imuOutput_.qBW().w() <<std::endl;
          myfile.close();
        #endif

        gotFirstMessages_ = true;
      }
    }
  }
};

}


#endif /* ROVIO_ROVIONODE_HPP_ */
