/*
The CVUIROSCmdVelPublisher class creates a graphical user interface featuring
five buttons to move the robot forward, back, left, right and stop.
The program publishes cmd_vel messages (geometry_msgs::Twist) and
displays odometry and robot info.

Author: Panisa Kraiwattanapong (based on Roberto Zegers)
Date: September 2025
License: BSD-3-Clause
*/

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#include <cmath>
#include <geometry_msgs/Twist.h>
#include <iomanip>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <ros/ros.h>
#include <sstream>
#include <std_srvs/Trigger.h>

class CVUIROSCmdVelPublisher {
public:
  CVUIROSCmdVelPublisher();
  void run();

private:
  ros::NodeHandle nh_;
  ros::Publisher twist_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber robot_info_sub_;

  geometry_msgs::Twist twist_msg_;
  nav_msgs::Odometry odom_msg_;
  std::string info_[6] = {"N/A", "N/A", "N/A", "N/A", "N/A", "N/A"};

  float linear_velocity_step_ = 0.1;
  float angular_velocity_step_ = 0.1;
  const std::string WINDOW_NAME_ = "CVUI ROS TELEOP";

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void infoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
};

CVUIROSCmdVelPublisher::CVUIROSCmdVelPublisher() {
  // Publisher
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cooper_1/cmd_vel", 10);

  // Subscribers
  odom_sub_ = nh_.subscribe("cooper_1/odom", 10,
                            &CVUIROSCmdVelPublisher::odomCallback, this);
  robot_info_sub_ = nh_.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      "robot_info", 10, &CVUIROSCmdVelPublisher::infoCallback, this);
}

void CVUIROSCmdVelPublisher::odomCallback(
    const nav_msgs::Odometry::ConstPtr &msg) {
  odom_msg_ = *msg;
}

void CVUIROSCmdVelPublisher::infoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  info_[0] = msg->data_field_01;
  info_[1] = msg->data_field_02;
  info_[2] = msg->data_field_03;
  info_[3] = msg->data_field_04;
  info_[4] = msg->data_field_05;
  info_[5] = msg->data_field_06;
}

void CVUIROSCmdVelPublisher::run() {
  cv::Mat frame(600, 350, CV_8UC3);
  cv::namedWindow(WINDOW_NAME_);
  cvui::init(WINDOW_NAME_);

  while (ros::ok()) {
    ros::spinOnce();
    frame = cv::Scalar(49, 52, 49); // background

    // --------------------- Robot Info ---------------------
    cvui::window(frame, 50, 20, 250, 170, "Robot Info:");
    for (int i = 0; i < 6; ++i) {
      cvui::printf(frame, 60, 50 + i * 20, 0.4, 0xff0000, "%s",
                   info_[i].c_str());
    }

    // --------------------- Buttons ---------------------
    if (cvui::button(frame, 125, 200, "Forward")) {
      twist_msg_.linear.x += linear_velocity_step_;
      twist_pub_.publish(twist_msg_);
    }

    if (cvui::button(frame, 125, 250, "Stop")) {
      twist_msg_.linear.x = 0.0;
      twist_msg_.angular.z = 0.0;
      twist_pub_.publish(twist_msg_);
    }

    if (cvui::button(frame, 125, 300, "Backward")) {
      twist_msg_.linear.x -= linear_velocity_step_;
      twist_pub_.publish(twist_msg_);
    }

    if (cvui::button(frame, 30, 250, "Left")) {
      twist_msg_.angular.z += angular_velocity_step_;
      twist_pub_.publish(twist_msg_);
    }

    if (cvui::button(frame, 250, 250, "Right")) {
      twist_msg_.angular.z -= angular_velocity_step_;
      twist_pub_.publish(twist_msg_);
    }

    // --------------------- Velocity Display ---------------------
    cvui::window(frame, 30, 350, 120, 60, "Linear velocity:");
    cvui::printf(frame, 40, 370, 0.4, 0xff0000, "%.2f m/s",
                 twist_msg_.linear.x);

    cvui::window(frame, 185, 350, 120, 60, "Angular velocity:");
    cvui::printf(frame, 195, 370, 0.4, 0xff0000, "%.2f rad/s",
                 twist_msg_.angular.z);

    // --------------------- Odometry Display ---------------------
    cvui::window(frame, 10, 420, 80, 80, "X");
    cvui::printf(frame, 30, 450, 0.4, 0xff0000, "%.2f",
                 odom_msg_.pose.pose.position.x);

    cvui::window(frame, 100, 420, 80, 80, "Y");
    cvui::printf(frame, 120, 450, 0.4, 0xff0000, "%.2f",
                 odom_msg_.pose.pose.position.y);

    cvui::window(frame, 190, 420, 80, 80, "Z");
    cvui::printf(frame, 210, 450, 0.4, 0xff0000, "%.2f",
                 odom_msg_.pose.pose.position.z);

    // --------------------- Call Distance Service ---------------------
    std::string distance_message_ = "N/A";

    if (cvui::button(frame, 50, 520, "Call Distance")) {
      ros::ServiceClient client =
          nh_.serviceClient<std_srvs::Trigger>("get_distance");
      std_srvs::Trigger srv;
      if (client.call(srv)) {
       distance_message_ =
            srv.response.message; // store latest message

        ROS_INFO_STREAM("Distance traveled: " << srv.response.message);
        try {
          double distance = std::stod(srv.response.message);

          ROS_INFO_STREAM("Distance traveled: " << distance_message_);
          cvui::window(frame, 190, 500, 200, 100, "Distance in meters");
          cvui::printf(frame, 200, 530, 0.4, 0xff0000, "%s",
                       distance_message_.c_str());
        } catch (const std::invalid_argument &e) {
          cvui::printf(frame, 100, 510, 0.4, 0xff0000, "Invalid value");
        }

      } else {

        ROS_ERROR("Failed to call service get_distance");
      }
    }

    cvui::window(frame, 190, 500, 200, 100, "Distance in meters");
    // cvui::printf(frame, 200, 530, 0.4, 0xff0000, "%s",
    //            distance_message_.c_str());

// --------------------- Reset Distance Service ---------------------
if (cvui::button(frame, 50, 560, "Reset Distance")) {
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("reset_distance");
    std_srvs::Trigger srv;
    if (client.call(srv)) {
        distance_message_ = "0.00"; // immediately reflect reset
        ROS_INFO("Distance reset successfully.");
    } else {
        distance_message_ = "Reset failed!";
        ROS_ERROR("Failed to call service reset_distance");
    }
}

    cvui::update();
    cv::imshow(WINDOW_NAME_, frame);

    if (cv::waitKey(20) == 27)
      break; // ESC
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cmd_vel_teleop_gui");
  CVUIROSCmdVelPublisher gui_node;
  gui_node.run();
  return 0;
}
