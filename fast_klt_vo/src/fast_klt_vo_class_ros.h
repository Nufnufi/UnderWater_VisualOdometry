
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include <cv_bridge/cv_bridge.h>
#include "vo_features.h"
#include <tf/transform_datatypes.h>

using namespace cv;
using namespace std;



#define MIN_NUM_FEAT 14
#define M_PI            3.14159265358979323846  /* pi */


class fast_klt_vo
{
public:

    // Constructor for node
    fast_klt_vo()
    {      
        ros::NodeHandle nh;

        //Start position and rotation from cave uw dataset
        nh.param<double>("/vo/start_x", this->start_x, -0.001058);
        nh.param<double>("/vo/start_y", this->start_y, -0.000489);
        nh.param<double>("/vo/start_z", this->start_z, 13.948603);
        nh.param<double>("/vo/start_rot_x", this->rot_x, -0.027858);
        nh.param<double>("/vo/start_rot_y", this->rot_y, 0.032963);
        nh.param<double>("/vo/start_rot_z", this->rot_z, -0.976291);
        nh.param<double>("/vo/start_rot_w", this->rot_w, 0.212118);

        //Parameters for camera
        nh.param<double>("/camera/focal", this->focal, 405.6385);   //718
        nh.param<double>("/camera/xpp", this->xpp, 189.9054);       //607.1928
        nh.param<double>("/camera/ypp", this->ypp, 139.915);        //185.2157
        nh.param<double>("/camera/scale", this->scale, 1);
        nh.param<double>("/camera/scale_threshold", this->scale_threshold, 0.1);
        nh.param<string>("/camera/tf_frame", this->tf_frame, "camera");

        //Parameters for KLT algorithm
        nh.param<double>("/vo/min_feature_recalculate", this->min_feature_recalculate, 14);   //Set min number of feature for redetection
        nh.param<double>("/vo/min_feature", this->min_feature, 6);                            //Set min number of features necessary to compute the essential matrix

        //Parameters for RANSAC
        nh.param<double>("/vo/ransac_prob", this->ransac_prob, 0.999);
        nh.param<double>("/vo/ransac_threshold", this->ransac_threshold, 1.0);

       // Read ROS parameter for path to image
        nh.param<string>("/camera/image", _groud_truth_file, "/home/maxime/Datasets/KITTI_VO/ground_poses/00.txt");
        nh.param<string>("/image/path_to_images", _images_path, "/home/maxime/Datasets/uwcaves/frames/frame_%06d.png");

        // introduce publishers
        visual_odometry_pub = nh.advertise<geometry_msgs::PoseStamped>("/visualodometry", 10);

        // introduce subscribers to camera image and ground truth odometry
        camera_sub = nh.subscribe("/camera/image_raw", 10, &fast_klt_vo::cameraCallback, this);
        truth_odometry_sub = nh.subscribe("/odometry", 10, &fast_klt_vo::truth_odometryCallback, this);
        imu_sub = nh.subscribe("/imu_adis_ros", 10, &fast_klt_vo::imuCallback, this);

        //Convert start rotation from Quaternion to rad
        tf::Quaternion Q;
        Q = tf::Quaternion(rot_x,rot_y,rot_z,rot_w);
        cout<<"Q: x = "<<Q.x()<<", y = "<<Q.y()<<", z = "<<Q.z()<<", w = "<< Q.w()<<endl;
        tf::Matrix3x3 m(Q);
        m.getRPY(start_rot_x, start_rot_y, start_rot_z);
        cout<<"Rotation (in rad): x = "<<start_rot_x<<", y = "<<start_rot_y<<", z = "<<start_rot_z<<endl;

        init_counter = 0;
        init_flag = false;
        new_image = false;

        //Ground truth initialization
        truth_pose.pose.position.x = start_x;
        truth_pose.pose.position.y = start_y;
        truth_pose.pose.position.z = start_z;
        truth_x_prev = start_x;
        truth_y_prev = start_y;
        truth_z_prev = start_z;

        //Computation time initialization
        min_elapsed_time = 555555;
        max_elapsed_time = 0.0001;
;
        ros::Time::init();
        ros::Duration(0.1).sleep();
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void start()
    {
        ros::Rate loop_rate(100);

        Mat img_1, img_2;   //Matrices to store the images

        //Create results document
        ofstream myfile;
        myfile.open ("results1_1.txt");

        char text[150];
        char text2[150];
        int fontFace = FONT_HERSHEY_PLAIN;
        double fontScale = 1;
        int thickness = 1;
        cv::Point textOrg(10, 40);
        cv::Point textOrg2(10, 80);

        //Wait the first two frames from the camera (Color)
        while(!init_flag){
            ros::spinOnce();
            loop_rate.sleep();
            if ( !img_1_c.data || !img_2_c.data ) {
              ROS_ERROR("Camera does not send any image");
            }
        }

        //Conversion to greyscale images
        cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
        cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

        //Feature detection, tracking for the first 2 images
        vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
        featureDetection(img_1, points1);        //detect features in img_1
        vector<uchar> status;
        featureTracking(img_1,img_2,points1,points2, status); //track those features to img_2

        //Recovering the pose and the essential matrix
        cv::Point2d pp(xpp, ypp); //Convert principle point of the camera x, y coordinates to cv::Point
        Mat E, R, t, mask;
        E = findEssentialMat(points2, points1, focal, pp, RANSAC, ransac_prob, ransac_threshold, mask); //Calculate essential matrix for corresponding points in both images
        recoverPose(E, points2, points1, R, t, focal, pp, mask);

        //Initialize translation and rotation matrices to start position
        t.at<double>(0,0) = start_x;
        t.at<double>(1,0) = start_y;
        t.at<double>(2,0) = start_z;
        //Quaternion to rotional matrix formulas can be found at http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
        R.at<double>(0,0) = 1 - 2*rot_y*rot_y - 2*rot_z*rot_z;
        R.at<double>(0,1) = 2*rot_x*rot_y - 2*rot_z*rot_w;
        R.at<double>(0,2) = 2*rot_x*rot_z + 2*rot_y*rot_w;
        R.at<double>(1,0) = 2*rot_x*rot_y + 2*rot_z*rot_w;
        R.at<double>(1,1) = 1 - 2*rot_x*rot_x - 2*rot_z*rot_z;
        R.at<double>(1,2) = 2*rot_y*rot_z - 2*rot_x*rot_w;
        R.at<double>(2,0) = 2*rot_x*rot_z - 2*rot_y*rot_w;
        R.at<double>(2,1) = 2*rot_y*rot_z + 2*rot_x*rot_w;
        R.at<double>(2,2) = 1 - 2*rot_x*rot_x - 2*rot_y*rot_y;

        Mat prevImage = img_2; //Keep previous image
        Mat currImage;
        vector<Point2f> prevFeatures = points2; //Keep features from previous image
        vector<Point2f> currFeatures, copy_prevFeatures;

        //Initialize final rotation and translation matrices
        R_f = R.clone();
        t_f = t.clone();
        //cout<<"Initialization t_f = "<<t_f<<endl;
        //cout<<"Initialization R_f = "<<R_f<<endl;

        //Initialize display windows
        namedWindow("Road facing camera", WINDOW_AUTOSIZE);// Create a window for display.
        namedWindow("Trajectory on x/y axis", WINDOW_AUTOSIZE);// Create a window for display.
        namedWindow("Trajectory on x/z axis", WINDOW_AUTOSIZE);// Create a window for display.
        Mat traj = Mat::zeros(1000, 1000, CV_8UC3);
        Mat traj2 = Mat::zeros(1000, 1000, CV_8UC3);
        int counter;
        counter = 0;

        while (ros::ok())
            {
            //if new image is detected
            if(new_image == true){
                new_image = false;  //Reset flag
                last_call = ros::Time::now();   //Start timmer

                //Load current image (color)
                Mat currImage_c = latest_image.clone();        //Clone latest image detected
                cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);   //Convert to greyscale
                vector<uchar> status;

                //Load latest ground truth
                geometry_msgs::PoseWithCovariance latest_ground_truth = truth_pose;

                //Feature detection and tracking
                copy_prevFeatures = prevFeatures;
                featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

                if(currFeatures.size() > min_feature){   //If enough features have been detected and tracked, compute the essential matrix
                    E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, ransac_prob, ransac_threshold, mask);
                    recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

                    Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);
                    for(int i=0;i<prevFeatures.size();i++)	{   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
                              prevPts.at<double>(0,i) = prevFeatures.at(i).x;
                              prevPts.at<double>(1,i) = prevFeatures.at(i).y;
                              currPts.at<double>(0,i) = currFeatures.at(i).x;
                              currPts.at<double>(1,i) = currFeatures.at(i).y;
                    }
                    scale = getAbsoluteScale((double)latest_ground_truth.pose.position.x,(double)latest_ground_truth.pose.position.y,(double)latest_ground_truth.pose.position.z);
                    cout << "Scale is " << scale << endl;

                    //Check if scale is bigger than threshold
                    if ((scale>scale_threshold)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

                        //Update ration/translation matrices
                        t_f = t_f + scale*(R_f*t);
                        R_f = R*R_f;

                        //Publish the odometry
                        geometry_msgs::PoseStamped current_pose;
                        current_pose.header.stamp =  ros::Time::now();
                        current_pose.header.frame_id = tf_frame;
                        current_pose.pose.position.x = (float)t_f.at<double>(0);
                        current_pose.pose.position.y = (float)t_f.at<double>(1);
                        current_pose.pose.position.z = (float)t_f.at<double>(2);
                        current_pose.pose.orientation.w = (float)(sqrt( std::max(0.0, (1.0 + R_f.at<double>(0,0) + R_f.at<double>(1,1) + R_f.at<double>(2,2)))) / 2.0);
                        current_pose.pose.orientation.x = (float)(sqrt( std::max(0.0, (1.0 + R_f.at<double>(0,0) - R_f.at<double>(1,1) - R_f.at<double>(2,2)))) / 2.0);
                        current_pose.pose.orientation.y = (float)(sqrt( std::max(0.0, (1.0 - R_f.at<double>(0,0) + R_f.at<double>(1,1) - R_f.at<double>(2,2)))) / 2.0);
                        current_pose.pose.orientation.z = (float)(sqrt( std::max(0.0, (1.0 - R_f.at<double>(0,0) - R_f.at<double>(1,1) + R_f.at<double>(2,2)))) / 2.0);
                        current_pose.pose.orientation.x = copysign(current_pose.pose.orientation.x, R_f.at<double>(2,1) - R_f.at<double>(1,2));
                        current_pose.pose.orientation.y = copysign(current_pose.pose.orientation.y, R_f.at<double>(0,2) - R_f.at<double>(2,0));
                        current_pose.pose.orientation.z = copysign(current_pose.pose.orientation.z, R_f.at<double>(1,0) - R_f.at<double>(0,1));
                        visual_odometry_pub.publish(current_pose);
                        ros::spinOnce();
                        loop_rate.sleep();
                        ROS_INFO("Odometry updated");
                    }
                    else{
                        ROS_WARN("scale below %f, or incorrect translation",scale_threshold);
                     }

                    //Update previous image and features
                    prevImage = currImage.clone();
                    prevFeatures = currFeatures;
                    ros::spinOnce();
                    loop_rate.sleep();

                    //Display trajectory
                    int display_start_x = 600;
                    int display_start_y = 200;
                    int display_start_z = 200;
                    int x = int(t_f.at<double>(0)*5) + display_start_x;
                    int y = int(t_f.at<double>(1)*5) + display_start_y;
                    int z = int(t_f.at<double>(2)*5) + display_start_z;
                    int x_truth = int(latest_ground_truth.pose.position.x*5) + display_start_x;
                    int y_truth = int(latest_ground_truth.pose.position.y*5) + display_start_y;
                    int z_truth = int(latest_ground_truth.pose.position.z*5) + display_start_z;
                    cout<<"x computed = "<<t_f.at<double>(0)<<"  y_computed = "<<t_f.at<double>(1)<<endl;
                    cout<<"x truth = "<<latest_ground_truth.pose.position.x<<", y_truth = "<<latest_ground_truth.pose.position.y<<endl;

                    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);
                    circle(traj, Point(x_truth, y_truth) ,1, CV_RGB(0,255,0), 2);
                    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
                    sprintf(text, "Estimated pose (Red): x = %fm, y = %fm ", t_f.at<double>(0), t_f.at<double>(1));
                    sprintf(text2,"Truth pose (Green) : x_truth = %fm, y_truth = %fm", latest_ground_truth.pose.position.x, latest_ground_truth.pose.position.y);
                    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
                    putText(traj, text2, textOrg2, fontFace, fontScale, Scalar::all(255), thickness, 8);

                    circle(traj2, Point(x, z) ,1, CV_RGB(255,0,255), 2);
                    circle(traj2, Point(x_truth, z_truth) ,1, CV_RGB(0,0,255), 2);
                    rectangle( traj2, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
                    sprintf(text, "Estimated pose (Pink): x = %fm, z = %fm ", t_f.at<double>(0), t_f.at<double>(2));
                    sprintf(text2,"Truth pose (Blue) : x_truth = %fm, z_truth = %fm", latest_ground_truth.pose.position.x, latest_ground_truth.pose.position.z);
                    putText(traj2, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
                    putText(traj2, text2, textOrg2, fontFace, fontScale, Scalar::all(255), thickness, 8);

                    imshow( "Road facing camera", currImage_c );
                    imshow( "Trajectory on x/y axis", traj);
                    imshow( "Trajectory on x/z axis", traj2);
                    waitKey(1);

                    //Reset counter
                    counter = 0;

                    // Determine time that has passed since last call
                    ros::Time curr_call = ros::Time::now();
                    ros::Duration temp_dur = curr_call - last_call;
                    last_call = curr_call;
                    float elapsed_time = temp_dur.toSec();

                    if(elapsed_time < min_elapsed_time){
                        min_elapsed_time = elapsed_time;
                        min_num_features = currFeatures.size();
                    }
                    if(elapsed_time > max_elapsed_time){
                        max_elapsed_time = elapsed_time;
                        max_num_features = currFeatures.size();
                    }

                    //cout << "Execution time estimated to " << elapsed_time << "s" << endl;
                    cout << "Current min execution time = "<<min_elapsed_time << "s, current max execution time = "<<max_elapsed_time<<"s"<<endl;
                    cout << "Corresponding to min number of features = "<<min_num_features << ", max umber of features = "<<max_num_features<<endl;
                    //cout <<"R_t = "<< R_f << endl;
                    //cout <<"t_f = "<< t_f << endl;

                    ros::spinOnce();
                    loop_rate.sleep();
                }

                //Recovery strategy
                else{
                    prevFeatures = copy_prevFeatures; //Reload previous features
                    counter++;
                    ros::spinOnce();
                    loop_rate.sleep();

                    //If feature tracking does not recover within 5 frames (or not enough features detected in previous image), update the previous image
                    if((counter>5)||(prevFeatures.size() < min_feature)){
                        counter = 0; //Reset counter
                        prevImage = currImage.clone(); //Update previous image
                        featureDetection(prevImage, prevFeatures); //Redetect features in prevImage
                        cout<<"Reset completed, current image shows "<<prevFeatures.size()<<" features of interest"<<endl;
                    }

                }
            }
            ros::spinOnce();
            loop_rate.sleep();
        }//End while loop
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    double getAbsoluteScale(double x, double y, double z){
      double x_prev, y_prev, z_prev;

      //Load previous truth values
      x_prev = truth_x_prev;
      y_prev = truth_y_prev;
      z_prev = truth_z_prev;

      //Update previous truth values
      truth_x_prev = x;
      truth_y_prev = y;
      truth_z_prev = z;

      return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

    }

    //////////////////////////////////////////////
    // callback function for the estimated linear and angular velocities
    void cameraCallback(const sensor_msgs::Image::ConstPtr& msg_in)
    {
        time_latest_image = msg_in->header.stamp;
        latest_image = cv_bridge::toCvCopy( msg_in)->image;
        new_image = true;

        //Keep the first 2 frames for initialization
        if(init_counter == 0){
            img_1_c = latest_image;
            init_counter++;
        }
        if(init_counter == 1){
            img_2_c = latest_image;
            init_counter++;
            init_flag=true;
        }

        return;
    }

    // Ground truth callback function
    void truth_odometryCallback(const nav_msgs::Odometry::ConstPtr& msg_in)
    {
       //Store latest truth odometry with corresponding time stamp
       time_truth_pose = msg_in->header.stamp;
       truth_pose = msg_in->pose;
       return;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg_in)
    {
        //Store latest imu information with corresponding time stamp
        time_imu = msg_in->header.stamp;
        imu_data.orientation = msg_in->orientation;
        imu_data.angular_velocity = msg_in->angular_velocity;
        imu_data.linear_acceleration = msg_in->linear_acceleration;
        return;
    }

protected:

    // Subscriber and Publisher
    ros::Subscriber camera_sub;
    ros::Subscriber truth_odometry_sub;
    ros::Subscriber imu_sub;
    ros::Publisher visual_odometry_pub;

    //Variable for initialization
    int init_counter;
    bool init_flag;
    Mat img_1_c, img_2_c;
    double rot_x, rot_y, rot_z, rot_w;

    //Parameters for the camera
    double scale;
    double scale_threshold;
    double focal;
    double xpp;
    double ypp;
    string tf_frame;

    //Variables for position and rotation
    double start_x;
    double start_y;
    double start_z;
    double start_rot_x;
    double start_rot_y;
    double start_rot_z;

    //Parameters for reading ground truth
    double truth_x_prev, truth_y_prev, truth_z_prev;

    //Parameters for the algorithm
    Mat R_f, t_f;                   //The final rotation and tranlation vectors
    double min_feature;
    double min_feature_recalculate;

    //Parameters for RANSAC
    double ransac_prob;
    double ransac_threshold;

    //Parameters for path to files
    string _groud_truth_file;
    string _images_path;

    //Parameters for min/max computation time
    float min_elapsed_time;
    double min_num_features;
    float max_elapsed_time;
    double max_num_features;

    //Variables to store latest image received, ground truth and IMU information
    Mat latest_image;
    ros::Time time_latest_image;
    bool new_image;
    geometry_msgs::PoseWithCovariance truth_pose;
    ros::Time time_truth_pose;
    sensor_msgs::Imu imu_data;
    ros::Time time_imu;
    ros::Time last_call;
};
