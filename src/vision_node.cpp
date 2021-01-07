/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Job van Dieten. */

// ROS HEADERS
#include <cv_bridge/cv_bridge.h>
#include <laundryman/CubeInfo.h>
#include <laundryman/CubeInfoList.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV headers
#include "opencv2/core.hpp"

// Libtorch headers
#include <torch/script.h>
#include <torch/torch.h>

class vision_node {
public:
  ros::NodeHandle nh;
  ros::Subscriber image_sub;
  ros::Publisher cube_info_list_pub;
  torch::jit::script::Module module;

  void imageCB(const laundryman::CubeInfoList &msg);
  void classify(cv::Mat &image, int &color_id, int &category_id);
  void init();
};

void vision_node::init() {
  image_sub = nh.subscribe("/aruco_node/cube_info_list", 1,
                           &vision_node::imageCB, this);
  torch::jit::script::Module module =
      torch::jit::load("/home/user/catkin_ws/src/laundryman/src/clothNet.pt");
  cube_info_list_pub =
      nh.advertise<laundryman::CubeInfoList>("/vision_node/cube_info_list", 1);
}

void vision_node::imageCB(const laundryman::CubeInfoList &msg) {
  laundryman::CubeInfoList cube_info_list;
  for (unsigned int i = 0; i < msg.data.size(); i++) {
    int color_id = 0;
    int category_id = 0;
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg.data[i].image,
                                   sensor_msgs::image_encodings::RGB8);
    } catch (...) {
      ROS_INFO("unrecognized image encoding");
    }
    if (cv_ptr != NULL) {
      cv::Mat inImage = cv_ptr->image;
      classify(inImage, color_id, category_id);
    }

    if (category_id == 1) {
      ROS_INFO("inflating msg");
      laundryman::CubeInfo cube_info;
      cube_info.id = msg.data[i].id;
      cube_info.color_id = color_id;
      cube_info.pose = msg.data[i].pose;
      cube_info_list.data.push_back(cube_info);
    }
  }

  cube_info_list_pub.publish(cube_info_list);
}

void vision_node::classify(cv::Mat &image, int &color_id, int &category_id) {
  /*Input
  image       --  homography camera image of cloth
  module      --  loaded tracing model
Output
  color_id    --  infered most likely color. Look up in the colorName_id table.
  category_id --  infered most likely category. Look up in the categoryName_id
table.
*/
  // cv::cvtColor(image, image, CV_BGR2RGB); // according to aruco_node.cpp, the
  // image received seems to be already RGB8
  cv::resize(
      image, image,
      cv::Size(60, 80)); // the trainingset is sized 60x80 to achive optimal
                         // classification. To be decided if resize or rect.
  // if the camera image is similar to trainingset, then resize. If not, rect to
  // similar shape.

  torch::Tensor img_tensor =
      torch::from_blob(image.data, {1, image.rows, image.cols, 3},
                       torch::kByte); // Convert Mat to Tensor
  img_tensor =
      img_tensor.permute({0, 3, 1, 2}); // Required by pytorch [C,H,W]. OpenCV
                                        // is [H,W,C]. output size [1,3,80,60]
  img_tensor = img_tensor.toType(
      torch::kFloat);               // Change data types kFloat is torch.float64
  img_tensor = img_tensor.div(255); // rescale pixel between 0 and 1

  // clothNet.pt is Torch Script via tracing. Load the torchscript model
  torch::jit::script::Module module = torch::jit::load(
      "/home/user/catkin_ws/src/laundryman/src/clothNet.pt"); // load the model
                                                              // in src
  // std::cout << "Inference model loaded successfully" << std::endl;
  auto output = module.forward({img_tensor});
  auto opt_dict = output.toGenericDict(); // genericDict
  auto color_score = opt_dict.at("color");
  // std::cout << color_score << std::endl; // CPUFloatType
  auto category_score = opt_dict.at("category");
  // std::cout << category_score << std::endl;
  std::tuple<torch::Tensor, torch::Tensor> max_classes_color =
      torch::max(color_score.toTensor(), 1);
  std::tuple<torch::Tensor, torch::Tensor> max_classes_category =
      torch::max(category_score.toTensor(), 1);
  color_id = std::get<1>(max_classes_color)[0].item<int>(); // CPULongType
  category_id = std::get<1>(max_classes_category)[0].item<int>();

  ROS_INFO("color: %d", color_id);
  ROS_INFO("category: %d", category_id);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vision_node");
  vision_node ts;
  ts.init();
  // suscribe aruco marker from aruco ros
  // ros::Subscriber sub = nh.subscribe("/aruco_single/pose", 1000,
  // pose_callback);
  ros::spin();
}