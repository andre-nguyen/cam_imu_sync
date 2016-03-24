/*
 * Copyright [2015] [Ke Sun  sunke.polyu@gmail.com]
 *                  [Chao Qu quchao@seas.upenn.edu]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mutex>

#include <cam_imu_sync/CamImuSynchronizer.h>
#include <pointgrey_camera_driver/PointGreyConfig.h>
#include <pointgrey_camera_driver/PointGreyCamera_ros.h>
#include <imu_vn_100/imu_vn_100.h>

namespace cam_imu_sync {

CamImuSynchronizer::CamImuSynchronizer(const ros::NodeHandle& pnh)
    : pnh_(pnh), imu_nh_(pnh, "imu"), cam_cfg_server_(pnh_) {
  // Initialize imu
  imu_ = boost::make_shared<Imu>(imu_nh_);
  imu_->Stream(true);

  // Initialize cameras
  int num_cameras = 0;
  if (!pnh_.getParam("num_cameras", num_cameras)) {
    throw std::runtime_error("Number of cameras is not specified");
  }

  ROS_INFO("Initializing %d cameras.", num_cameras);
  for (int i = 0; i < num_cameras; ++i) {
    const auto prefix = "cam" + std::to_string(i);
    cameras_.push_back(boost::make_shared<Cam>(pnh_, prefix));
  }

  // Initialize reconfigure server
  cam_cfg_server_.setCallback(
      boost::bind(&CamImuSynchronizer::configure, this, _1, _2));
}

CamImuSynchronizer::~CamImuSynchronizer()
{
  stopPoll();
}

void CamImuSynchronizer::configure(Config& config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", pnh_.getNamespace().c_str(),
             "Initializing reconfigure server");
  }

  if (is_polling_) stopPoll();
  // Configure cameras
  configureCameras(config);
  startPoll();
}

void CamImuSynchronizer::pollImages() {
  // Before the polling loop, we make sure that all camera buffers are cleared
  for (auto& cam : cameras_) {
    auto image_msg = boost::make_shared<sensor_msgs::Image>();
    cam->Grab(image_msg);
  }

  ros::Time prev_time;
  bool publish = true;
  while (is_polling_ && ros::ok()) {
    ros::Time time;
    publish = true;
    for (size_t i = 0; i < cameras_.size(); ++i) {
      auto image_msg = boost::make_shared<sensor_msgs::Image>();
      cameras_[i]->Grab(image_msg);
      // After the first camera finished grabing, we get the time stamp from
      // imu. Because Grab blocks until the buffer is retrieved, this time stamp
      // is guaranteed to correspond to the imu that triggered this image
      if (i == 0) {
          imu_->lock_sync_info();
          time = imu_->sync_info()->time;
          if (prev_time == time) {
              //ROS_ERROR("SHIT DOUBLE TIME STAMP %d:%d-%d:%d", prev_time.sec,
              //prev_time.nsec, time.sec, time.nsec);
              publish = false;
          }
          prev_time = time;
          imu_->unlock_sync_info();
      }
      image_msg->header.stamp = time;
      // Publish takes less then 0.1ms to finish, so it is safe to put it here
      // in the loop
      if(publish) cameras_[i]->Publish(image_msg);
    }
  }
}

void CamImuSynchronizer::configureCameras(Config& config) {
  config.frame_rate = imu_->sync_info()->rate;
  for (auto& cam : cameras_) {
    cam->Stop();
    // Using LEVEL_RECONFIGURE_CLOSE because we stopped the camera anyways
    cam->camera().setNewConfiguration(config, PointGreyCamera::LEVEL_RECONFIGURE_CLOSE);
    cam->set_fps(config.frame_rate);
    cam->Start();
  }
}

void CamImuSynchronizer::startPoll() {
  is_polling_ = true;
  img_poll_thread_ =
      boost::make_shared<boost::thread>(&CamImuSynchronizer::pollImages, this);
}

void CamImuSynchronizer::stopPoll() {
  if (!is_polling_) return;
  is_polling_ = false;
  img_poll_thread_->join();
}

}  // namespace cam_imu_sync
