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
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Libtorch headers
#include <torch/torch.h>
#include <torch/script.h>

//marker
#include <geometry_msgs/PoseStamped.h>

class vision_node
{
public:
	vision_node(ros::NodeHandle nh_);
	~vision_node();
	image_transport::ImageTransport _imageTransport;
	image_transport::Subscriber image_sub;
	

protected:
	void imageCB(const sensor_msgs::ImageConstPtr& msg);
	void ImageProcessing();
	void Classfier(cv::Mat &image);

private:
	cv::Mat img_bgr, img1, img2, thresh, diff;
	int i;
};

const std::string win1 = "Live Camera Feed";
const std::string win2 = "Threshold Difference";

vision_node::vision_node(ros::NodeHandle nh_): _imageTransport(nh_)
{
	image_sub = _imageTransport.subscribe("xtion/rgb/image_raw", 1, &vision_node::imageCB, this, image_transport::TransportHints("compressed"));
        
	cv::namedWindow(win1, CV_WINDOW_FREERATIO);
	cv::namedWindow(win2, CV_WINDOW_FREERATIO);
	i=0;
}

vision_node::~vision_node()
{
	cv::destroyWindow(win1);
	cv::destroyWindow(win2);
}

void vision_node::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cvPtr;
	try
	{ 
		cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) 
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cvPtr->image.copyTo(img_bgr);
		cv::cvtColor(cvPtr->image, img1, cv::COLOR_BGR2GRAY);
		this->ImageProcessing();


}




void vision_node::Classfier(cv::Mat &image){
	//https://pytorch.org/tutorials/advanced/cpp_export.html
    torch::Tensor img_tensor = torch::from_blob(image.data, {1, image.rows, image.cols, 3}, torch::kByte); //Convert Mat to Tensor
    img_tensor = img_tensor.permute({0, 3, 1, 2}); //Required by pytorch [C,H,W]. OpenCV is [H,W,C]. output size [1,3,80,60]
    img_tensor = img_tensor.toType(torch::kFloat); // Change data types
    img_tensor = img_tensor.div(255); // rescale pixel between 0 and 1

    torch::jit::script::Module module = torch::jit::load("clothNet.pt"); //clothNet.pt is Torch Script via tracing. Load the torchscript model
	torch::Tensor output = module.forward({img_tensor}).toTensor(); //In python the output is a dict of two keys 'color' 'category', in which is the score of each class, and the index of max score is the predicted class.
    // The following needs testing is output a map?dict? In general, output1 is category, output2 is color
	std::cout << output << std::endl;
	//auto max_result = output.max(1, true);
    //auto max_index = std::get<1>(max_result).item<float>();
    //std::cout << max_index << std::endl;

}

//handle images here
void vision_node::ImageProcessing()
{
	// Aruco marker recognition, detect the location of the target object
	// find the nearby cloth image, classify if it is apparel, if so, sent the information to the action node.
	if(i!=0)
	{	

	cv::imshow(win1, img_bgr);

    cv::Mat graymat;
    cv::cvtColor(img_bgr, graymat, cv::COLOR_BGR2GRAY);

    cv::imshow(win2, graymat);

	// aruco marker detection

	// process the cloth image to the size 80x60x3 (Convert OpenCV default BGR to Torch RGB)


	// classify the category and color of the image
	//vision_node::Classfier(image_cloth);

	}
	++i;
	cv::waitKey(1);
}



//
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::cout << "Position x: " << msg->pose.position.x << std::endl;
    std::cout << "Position y: " << msg->pose.position.y << std::endl;
    std::cout << "Position z: " << msg->pose.position.z << std::endl;

    std::cout << "Orientation x: " << msg->pose.orientation.x << std::endl;
    std::cout << "Orientation y: " << msg->pose.orientation.y << std::endl;
    std::cout << "Orientation z: " << msg->pose.orientation.z << std::endl;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_node");
	ros::NodeHandle nh;
	vision_node ts(nh);
	//suscribe aruco marker from aruco ros
    ros::Subscriber sub = nh.subscribe("/aruco_single/pose", 1000, pose_callback);
	ros::spin();
}


#if 0
#include <ros/ros.h>

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "vision_node");

    // Create a ROS node handle
  ros::NodeHandle nh;


  ROS_INFO("Hello, World!");

  // Don't exit the program.
  ros::spin();
}

#endif
