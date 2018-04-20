/*

The MIT License

Copyright (c) 2015 Avi Singh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "ros/ros.h"
#include <string.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
using namespace cv;
using namespace std;

class pose_display_ros
{
public:

    // Constructor for node
    pose_display_ros()
    {
        ros::NodeHandle nh;

        //ROS parameters
        nh.param<string>("/pose_display_ros/topic_estimated_pose", this->topic_estimated_pose, "/orbslam_pose");
        nh.param<string>("/pose_display_ros/topic_groundtruth_pose", this->topic_ground_truth_pose, "/odometry");
        nh.param<int>("/pose_display_ros/display_x", this->display_x, 800);
        nh.param<int>("/pose_display_ros/display_y", this->display_y, 800);
        nh.param<int>("/pose_display_ros/zoom_coeff", this->zoom_coeff, 5);

        //Introduce subscribers to estimated odometry and ground truth pose
        estimated_odo_sub = nh.subscribe(topic_estimated_pose, 10, &pose_display_ros::estimated_odoCallback, this);
        truth_odometry_sub = nh.subscribe(topic_ground_truth_pose, 10, &pose_display_ros::truth_odometryCallback, this);
        scale_pub = nh.advertise<geometry_msgs::PoseStamped>("/euclidean_error", 10);

        //Initialize flags
        flag_est = false;
        flag_gt = false;

        //Initialize prev pose
        prev_x_ground = 0;
        prev_y_ground = 0;
        prev_x_est = 0;
        prev_y_est = 0;

        ros::Time::init();
        ros::Duration(0.1).sleep();
    }

    int start(){
        ros::Rate loop_rate(10); // 10 hz
        namedWindow("Truth and estimated trajectory on x/y axis", WINDOW_AUTOSIZE);// Create a window for display.
        cv::Point textOrg(10, 40);
        cv::Point textOrg2(10, 80);
        Mat traj = Mat::zeros(display_x, display_y, CV_8UC3);
        char text[150],text2[150];
        int fontFace = FONT_HERSHEY_PLAIN;
        double fontScale = 1;
        int thickness = 1;
        int display_x_ground, display_y_ground, display_est_x, display_est_y;
        float scale_gt, scale_est, ratio;
        bool b1, b2;
        b1 = false;
        b2 = false;

        while(ros::ok()){

            if(flag_est){//If pose estimated
                flag_est = false;
                display_est_x = (int)(zoom_coeff*est_x + 3*display_x/4);
                display_est_y = (int)(zoom_coeff*est_y + display_y/2);
                circle(traj, Point(display_est_x, display_est_y) ,1, CV_RGB(255,0,0), 2);
                rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
                sprintf(text2, "Estimated x/y pose (Red)");
                putText(traj, text2, textOrg2, fontFace, fontScale, Scalar::all(255), thickness, 8);

                scale_est = sqrt((est_x - prev_x_est)*(est_x - prev_x_est) + (est_y - prev_y_est)*(est_y - prev_y_est) + (est_z - prev_z_est)*(est_z - prev_z_est));
                b1 = true;

                //Update previous estimated pose
                prev_x_est = est_x;
                prev_y_est = est_y;
                prev_z_est = est_z;

            }

            //Display ground truth
            if(flag_gt && (abs(ground_x - prev_x_ground) > 0.01)){//If ground truth pose was updated
                flag_gt = false;
                display_x_ground = (int)(zoom_coeff*ground_x + 3*display_x/4);
                display_y_ground = (int)(zoom_coeff*ground_y + display_y/2);
                circle(traj, Point(display_x_ground, display_y_ground) ,1, CV_RGB(0,255,0), 2);
                rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
                sprintf(text, "Ground truth x/y pose (Green)");
                putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
                imshow( "Truth and estimated trajectory on x/y axis", traj);
                waitKey(1);

                scale_gt = sqrt((ground_x - prev_x_ground)*(ground_x - prev_x_ground) + (ground_y - prev_y_ground)*(ground_y - prev_y_ground) + (ground_z - prev_z_ground)*(ground_z - prev_z_ground));
                b2 = true;

                //Update previous ground truth
                prev_x_ground = ground_x;
                prev_y_ground = ground_y;
                prev_z_ground = ground_z;

                }

            if(b1 && b2){
                b1 = false;
                b2 = false;

                ROS_INFO("Truth pose xtruth = %f, ytruth = %f", ground_x , ground_y);
                ROS_WARN("Estimated pose xest = %f, yest = %f", est_x, est_y);
                ROS_ERROR("Scale_GT = %f, Scale_est = %f, ratio scale_est/scale_GT = %f ", scale_gt, scale_est, ratio);

                //Display error between groun truth x/y and estimated x/y coordinates
                geometry_msgs::PoseStamped scale;
                scale.header.stamp = ros::Time::now();
                scale.pose.position.x = scale_est;  //Display estimated scale
                scale.pose.position.y = scale_gt;   //Display ground truth scale
                ratio = scale_est/scale_gt;
                scale.pose.position.z = ratio;      //Display ratio estimated scale/ gt scale
                scale_pub.publish(scale);
           }
           ros::spinOnce();
          loop_rate.sleep();
        }
    }



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Ground truth posecallback function
    void truth_odometryCallback(const nav_msgs::Odometry::ConstPtr& msg_in)
    {
       ground_x = msg_in->pose.pose.position.x;
       ground_y = msg_in->pose.pose.position.y;
       ground_z = msg_in->pose.pose.position.z;
       flag_gt = true;
       //ROS_ERROR("GT detected x = %d, y = %d",ground_x, ground_y);
       return;
    }

    // Estimated pose callback function
    void estimated_odoCallback(const nav_msgs::Odometry::ConstPtr& msg_in)
    {
       est_x = msg_in->pose.pose.position.x;
       est_y = msg_in->pose.pose.position.y;
       est_z = msg_in->pose.pose.position.z;
       flag_est = true;
       //ROS_ERROR("Estimated pose detected x = %f, y = %f",est_x, est_y);
       return;
    }

protected:

    // Subscriber and Publisher
    ros::Subscriber truth_odometry_sub;
    ros::Subscriber estimated_odo_sub;
    ros::Publisher scale_pub;

    //Ros parameters
    string path_to_image, image_extension;
    string topic_ground_truth_pose, topic_estimated_pose;
    int display_x,display_y, zoom_coeff;
    float prev_x_ground, prev_y_ground, prev_z_ground, prev_x_est, prev_y_est, prev_z_est;

    //Variables for the display
    float ground_x, ground_y, ground_z;
    float est_x ,est_y, est_z;
    bool flag_gt, flag_est;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_to_ros_converter");

  // initialize the node and subscribers
  pose_display_ros displayer;

  // start the publishing
  displayer.start();

  return 0;
}
