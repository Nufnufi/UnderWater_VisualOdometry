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
#include <sensor_msgs/image_encodings.h>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


using namespace cv;
using namespace std;

class image_converter_ros
{
public:

    // Constructor for node
    image_converter_ros()
    {
        ros::NodeHandle nh;

        //Start position and rotation from cave uw dataset
        nh.param<int>("/image_converter_ros/start_image", this->start_image_id, 16994);
        nh.param<int>("/image_converter_ros/last_image", this->last_image_id, 17990);
        nh.param<string>("/image_converter_ros/path_to_image", this->path_to_image, "/home/maxime/Datasets/Downward_looking_camera_(GoPro15002)/100GOPRO/G00");
        nh.param<string>("/image_converter_ros/image_extension", this->image_extension, ".JPG");
        nh.param<double>("/image_converter_ros/fps", this->fps, 1);
        nh.param<string>("/image_converter_ros/topic_color", this->topic_pub_color, "/color");
        nh.param<string>("/image_converter_ros/topic_grey", this->topic_pub_grey, "/grey");

        // Introduce publishers
        image_color_pub = nh.advertise<sensor_msgs::Image>(topic_pub_color, 10);
        image_greyscale_pub = nh.advertise<sensor_msgs::Image>(topic_pub_grey, 10);

        ros::Time::init();
        ros::Duration(0.1).sleep();
    }

int start(){
    ros::Rate loop_rate(10); // 10 hz
    Mat currImage_c, currImage;
    string imageName = "";
    string numFrame_string = "";
    ros::Time start_time, end_time; //Variables to control the number of images published by second
    ros::Duration elapsed_time,time_to_wait;

    for(int numFrame = start_image_id ; numFrame < last_image_id; numFrame++){

        start_time = ros::Time::now();
        numFrame_string = static_cast<ostringstream*>( &(ostringstream() << numFrame) )->str(); //Conver numFrame to string
        //numFrame_string = to_string(numFrame);
        imageName = path_to_image + numFrame_string + image_extension;
        cout<<"Current image path: "<<imageName.c_str()<<endl;
        currImage_c = imread(imageName.c_str()); //Read current image
        if ( !currImage_c.data) {
          std::cout<< " --(!) Error reading images " << std::endl;
          return 0;
        }
        cvtColor(currImage_c, currImage, COLOR_BGR2GRAY); //Convert image to greyscale
        cv_bridge::CvImage color_out_msg;
        color_out_msg.encoding = sensor_msgs::image_encodings::RGB8; //TYPE_32FC1
        color_out_msg.image  = currImage_c;
        cv_bridge::CvImage grey_out_msg;
        grey_out_msg.encoding = sensor_msgs::image_encodings::MONO8; //TYPE_32FC1
        grey_out_msg.image = currImage;

        end_time = ros::Time::now();
        elapsed_time = end_time - start_time;
        time_to_wait =  ros::Duration(1/fps) - elapsed_time - time_to_wait;
        time_to_wait.sleep();

        start_time = ros::Time::now();
        color_out_msg.header.stamp = ros::Time::now();
        grey_out_msg.header.stamp = ros::Time::now();
        image_color_pub.publish(color_out_msg.toImageMsg());
        image_greyscale_pub.publish(grey_out_msg.toImageMsg());

        ros::spinOnce();
        loop_rate.sleep();
        end_time = ros::Time::now();
        time_to_wait = end_time - start_time;

        if (!ros::ok()){
            break;
        }

    }


}
protected:

    // Subscriber and Publisher
    ros::Publisher image_color_pub;
    ros::Publisher image_greyscale_pub;

    //Ros parameters
    int start_image_id,last_image_id;
    double fps;
    string path_to_image, image_extension;
    string topic_pub_color, topic_pub_grey;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_to_ros_converter");

  // initialize the node and subscribers
  image_converter_ros converter;

  // start the publishing
  converter.start();

  return 0;
}
