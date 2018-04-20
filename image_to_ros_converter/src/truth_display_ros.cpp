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
#include <math.h>       /* atan2 */
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
        nh.param<string>("/pose_display_ros/topic_groundtruth_pose", this->topic_ground_truth_pose, "/odometry");
        nh.param<int>("/pose_display_ros/display_x", this->display_x, 800);
        nh.param<int>("/pose_display_ros/display_y", this->display_y, 800);
        nh.param<int>("/pose_display_ros/zoom_coeff", this->zoom_coeff, 4);

        //Introduce subscribers to estimated odometry and ground truth pose
        truth_odometry_sub = nh.subscribe(topic_ground_truth_pose, 10, &pose_display_ros::truth_odometryCallback, this);
        flag_gt = false;

        ros::Time::init();
        ros::Duration(0.1).sleep();
    }

int start(){
    ros::Rate loop_rate(10); // 10 hz
    namedWindow("Truth trajectory on x/y axis", WINDOW_AUTOSIZE);// Create a window for display.
    cv::Point textOrg(10, 40);
    Mat traj = Mat::zeros(display_x, display_y, CV_8UC3);
    char text[150];
    int fontFace = FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;

    while(ros::ok()){

        //If new points detected
        if(flag_gt){
            flag_gt = false;
            ROS_INFO("Display trajectory xtruth = %d, ytruth = %d",(zoom_coeff*ground_x + 3*display_x/4), (zoom_coeff*ground_y + display_y/4));
            circle(traj, Point((zoom_coeff*ground_x + 3*display_x/4), (zoom_coeff*ground_y + display_y/4)) ,1, CV_RGB(0,255,0), 2);
            rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
            sprintf(text,"Truth x/y pose (Green)");
            putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
            imshow( "Truth trajectory on x/y axis", traj);
            waitKey(1);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Ground truth posecallback function
void truth_odometryCallback(const nav_msgs::Odometry::ConstPtr& msg_in)
{
   ground_x =(double)(msg_in->pose.pose.position.x);
   ground_y =(double)(msg_in->pose.pose.position.y);
   flag_gt = true;
   //ROS_ERROR("GT detected x = %d, y = %d",ground_x, ground_y);
   return;
}

protected:

    // Subscriber and Publisher
    ros::Subscriber truth_odometry_sub;

    //Ros parameters
    string topic_ground_truth_pose;
    int display_x,display_y, zoom_coeff;

    //Variables for the display
    int ground_x, ground_y;
    bool flag_gt;
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
