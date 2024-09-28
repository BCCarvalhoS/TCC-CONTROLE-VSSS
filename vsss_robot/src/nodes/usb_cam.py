#!/usr/bin/env python3

*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
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
 *   * Neither the name of the Robert Bosch nor the names of its
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
 *
 *********************************************************************/

#include <linux/videodev2.h>
#include <ros/ros.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "ros/service_server.h"
#include "usb_cam/usb_cam.h"

using namespace usb_cam;

bool UsbCam::create_suspended = false;

/* ROS */
ros::Timer* UsbCam::frame_timer = nullptr;
sensor_msgs::Image* UsbCam::img_msg = nullptr;
image_transport::CameraPublisher* UsbCam::image_pub = nullptr;
camera_info_manager::CameraInfoManager* UsbCam::camera_info = nullptr;
ros::ServiceServer* UsbCam::service_start = nullptr;
ros::ServiceServer* UsbCam::service_stop = nullptr;
ros::ServiceServer* UsbCam::service_supported_formats = nullptr;
ros::ServiceServer* UsbCam::service_supported_controls = nullptr;
image_transport::ImageTransport* UsbCam::image_transport = nullptr;

/* Node parameters */
std::string UsbCam::camera_name = "head_camera";
std::string UsbCam::camera_frame_id = "head_camera";
std::string UsbCam::camera_transport_suffix = "image_raw";
std::string UsbCam::camera_info_url = "";

/* ROS Service callback functions */
bool UsbCam::service_start_callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    return start_capture();
}

bool UsbCam::service_stop_callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    return suspend();
}

bool UsbCam::service_supported_formats_callback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
    get_supported_formats();
    std::stringstream output_stream;
    std::cout << "SUPPORTED INPUT FORMATS FOR V4L DEVICE " << video_device_name << std::endl;
    for(auto fmt : supported_formats)
    {
        output_stream << " | " << fmt.format.description
                      << " [" << fmt.interval.width << " x "
                      << fmt.interval.height << "], "
                      << fmt.interval.discrete.denominator / fmt.interval.discrete.numerator
                      << " fps";
        std::cout << "\t" << fmt.format.description
                  << " [" << fmt.interval.width << " x "
                  << fmt.interval.height << "], "
                  << fmt.interval.discrete.denominator / fmt.interval.discrete.numerator
                  << " fps" << std::endl;
    }
    response.success = true;
    response.message = output_stream.str();
    return true;
}

bool UsbCam::service_supported_controls_callback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
    std::stringstream output_stream;
    std::cout << "SUPPORTED V4L CONTROLS FOR DEVICE " << video_device_name << std::endl;
    std::cout << "NOTE: these controls are supported on host machine, not natively by ROS!" << std::endl;
    for(auto c: controls)
    {
        output_stream << " | " << c.name << " [" << c.value << "]: " << c.description;
        std::cout << c.name << " [" << c.value << "]" << std::endl << "\t" << c.description << std::endl;
    }
    response.success = true;
    response.message = output_stream.str();
    return true;
}
