
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <cv_bridge/cv_bridge.h>
#include "vo_features.h"
#include <tf/transform_datatypes.h>
#include "math.h"




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
        nh.param<double>("/vo/cov_x", this->cov_x, 0.2);
        nh.param<double>("/vo/cov_y", this->cov_y, 0.2);
        nh.param<double>("/vo/cov_z", this->cov_z, 0.2);
        nh.param<double>("/vo/cov_rot_x", this->cov_rot_x, 0.2);
        nh.param<double>("/vo/cov_rot_y", this->cov_rot_y, 0.2);
        nh.param<double>("/vo/cov_rot_z", this->cov_rot_z, 0.2);
        nh.param<double>("/vo/cov_imu_x", this->cov_imu_x, 0.1);
        nh.param<double>("/vo/cov_imu_y", this->cov_imu_y, 0.1);
        nh.param<double>("/vo/cov_imu_z", this->cov_imu_z, 0.1);

        //Parameters for the topics
        nh.param<string>("/vo/camera_topic_sub", this->camera_topic_sub,"/camera/image_raw");
        nh.param<string>("/vo/imu_topic_sub", this->imu_topic_sub,"/imu_xsens_mti_ros");
        nh.param<string>("/vo/groundtruth_topic_sub", this->groundtruth_topic_sub,"/odometry");


        //Parameters for camera
        nh.param<double>("/vo/focal", this->focal, 405.6385);   //718
        nh.param<double>("/vo/xpp", this->xpp, 189.9054);       //607.1928
        nh.param<double>("/vo/ypp", this->ypp, 139.915);        //185.2157
        nh.param<double>("/vo/scale", this->scale, 1);
        nh.param<string>("/vo/tf_frame", this->tf_frame, "base_footprint");


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
        ekf_vo_pub = nh.advertise<nav_msgs::Odometry>("/vo", 10);
        ekf_imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_data", 10);
        error_pub = nh.advertise<geometry_msgs::PoseStamped>("/euclidean_error", 10);

        // introduce subscribers to camera image and ground truth odometry
        camera_sub = nh.subscribe(camera_topic_sub, 10, &fast_klt_vo::cameraCallback, this);
        truth_odometry_sub = nh.subscribe(groundtruth_topic_sub, 10, &fast_klt_vo::truth_odometryCallback, this);
        imu_sub = nh.subscribe(imu_topic_sub, 10, &fast_klt_vo::imuCallback, this);
        ekf_odo_sub = nh.subscribe("/robot_pose_ekf/odom_combined", 10, &fast_klt_vo::ekfodoCallback, this);

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

        //Computation time initialization
        min_elapsed_time = 555555;
        max_elapsed_time = 0.0001;

        //Ground truth initialization
        truth_pose.pose.position.x = start_x;
        truth_pose.pose.position.y = start_y;
        truth_pose.pose.position.z = start_z;

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

        //Initialize display windows
        namedWindow("Road facing camera", WINDOW_AUTOSIZE);// Create a window for display.
        namedWindow("Trajectory on x/y axis", WINDOW_AUTOSIZE);// Create a window for display.
        namedWindow("Trajectory on x/z axis", WINDOW_AUTOSIZE);// Create a window for display.
        namedWindow("Trajectory on x/z axis", WINDOW_AUTOSIZE);// Create a window for display.
        Mat traj = Mat::zeros(1000, 1000, CV_8UC3);     //Display trajectory x/y axis
        Mat traj2 = Mat::zeros(1000, 1000, CV_8UC3);    //Display trajectory x/z axis

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

                //Keep in memory min and max number of features detected
                if(min_num_features > currFeatures.size()){
                    min_num_features = currFeatures.size();
                }
                if(max_num_features < currFeatures.size()){
                    max_num_features = currFeatures.size();
                }
                if(currFeatures.size() < 100){
                    ROS_ERROR("LESS THAN 100 FEATURES");
                }

                if(currFeatures.size() > min_feature){   //If enough features have been detected and tracked, compute the essential matrix
                    E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, ransac_prob, ransac_threshold, mask);
                    recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

                    Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);
                    for(int i=0;i<prevFeatures.size();i++){ //This (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
                              prevPts.at<double>(0,i) = prevFeatures.at(i).x;
                              prevPts.at<double>(1,i) = prevFeatures.at(i).y;
                              currPts.at<double>(0,i) = currFeatures.at(i).x;
                              currPts.at<double>(1,i) = currFeatures.at(i).y;
                    }

                    //Publish the visual odometry
                    nav_msgs::Odometry estimated_pose;
                    estimated_pose.header.stamp =  ros::Time::now();
                    estimated_pose.header.frame_id = tf_frame;
                    estimated_pose.pose.pose.position.x = (float)(t.at<double>(0) + t_f.at<double>(0,0));
                    estimated_pose.pose.pose.position.y = (float)(t.at<double>(1) + t_f.at<double>(1,0));
                    estimated_pose.pose.pose.position.z = (float)(t.at<double>(2) + t_f.at<double>(2,0));
                    estimated_pose.pose.pose.orientation.w = (float)(sqrt(std::max(0.0, (1.0 + R.at<double>(0,0) + R_f.at<double>(0,0) + R.at<double>(1,1) + R_f.at<double>(1,1) + R.at<double>(2,2) + R_f.at<double>(2,2)))) / 2.0);
                    estimated_pose.pose.pose.orientation.x = (float)(sqrt(std::max(0.0, (1.0 + R.at<double>(0,0) + R_f.at<double>(0,0) - R.at<double>(1,1) - R_f.at<double>(1,1) - R.at<double>(2,2) - R_f.at<double>(2,2)))) / 2.0);
                    estimated_pose.pose.pose.orientation.y = (float)(sqrt(std::max(0.0, (1.0 - R.at<double>(0,0) - R_f.at<double>(0,0) + R.at<double>(1,1) + R_f.at<double>(1,1) - R.at<double>(2,2) - R_f.at<double>(2,2)))) / 2.0);
                    estimated_pose.pose.pose.orientation.z = (float)(sqrt(std::max(0.0, (1.0 - R.at<double>(0,0) - R_f.at<double>(0,0) - R.at<double>(1,1) - R_f.at<double>(1,1) + R.at<double>(2,2) + R_f.at<double>(2,2)))) / 2.0);
                    estimated_pose.pose.pose.orientation.x = copysign(estimated_pose.pose.pose.orientation.x, R.at<double>(2,1) + R_f.at<double>(2,1) - R.at<double>(1,2) - R_f.at<double>(1,2));
                    estimated_pose.pose.pose.orientation.y = copysign(estimated_pose.pose.pose.orientation.y, R.at<double>(0,2) + R_f.at<double>(0,2) - R.at<double>(2,0) - R_f.at<double>(2,0));
                    estimated_pose.pose.pose.orientation.z = copysign(estimated_pose.pose.pose.orientation.z, R.at<double>(1,0) + R_f.at<double>(1,0) - R.at<double>(0,1) - R_f.at<double>(0,1));
                    estimated_pose.pose.covariance[0] = (float)cov_x;
                    estimated_pose.pose.covariance[7] = (float)cov_y;
                    estimated_pose.pose.covariance[14] = (float)cov_z;
                    estimated_pose.pose.covariance[21] = (float)cov_rot_x;
                    estimated_pose.pose.covariance[28] = (float)cov_rot_y;
                    estimated_pose.pose.covariance[35] = (float)cov_rot_z;
                    ekf_vo_pub.publish(estimated_pose);

                    ros::spinOnce();
                    loop_rate.sleep();
                    ROS_WARN("Odometry estimated");

                    //Update previous image and features
                    prevImage = currImage.clone();
                    prevFeatures = currFeatures;
                    ros::spinOnce();
                    loop_rate.sleep();

                    //Display trajectory
                    int display_start_x = 600;
                    int display_start_y = 800;
                    int display_start_z = 200;
                    double display_coeff = 2;
                    int x = int(t_f.at<double>(0)*display_coeff) + display_start_x;
                    int y = int(t_f.at<double>(1)*display_coeff) + display_start_y;
                    int z = int(t_f.at<double>(2)*display_coeff) + display_start_z;
                    int x_truth = int(latest_ground_truth.pose.position.x*display_coeff) + display_start_x;
                    int y_truth = int(latest_ground_truth.pose.position.y*display_coeff) + display_start_y;
                    int z_truth = int(latest_ground_truth.pose.position.z*display_coeff) + display_start_z;
                    cout<<"x computed = "<<t_f.at<double>(0)<<"  y_computed = "<<t_f.at<double>(1)<<endl;
                    cout<<"x truth = "<<latest_ground_truth.pose.position.x<<", y_truth = "<<latest_ground_truth.pose.position.y<<endl;

                    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);
                    circle(traj, Point(x_truth, y_truth) ,1, CV_RGB(0,255,0), 2);
                    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
                    sprintf(text, "Estimated x/y pose (Red)");
                    sprintf(text2,"Truth x/y pose (Green)");
                    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
                    putText(traj, text2, textOrg2, fontFace, fontScale, Scalar::all(255), thickness, 8);

                    circle(traj2, Point(x, z) ,1, CV_RGB(255,0,255), 2);
                    circle(traj2, Point(x_truth, z_truth) ,1, CV_RGB(0,0,255), 2);
                    rectangle( traj2, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
                    sprintf(text, "Estimated x/z pose (Pink)");
                    sprintf(text2,"Truth x/z pose (Blue)");
                    putText(traj2, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
                    putText(traj2, text2, textOrg2, fontFace, fontScale, Scalar::all(255), thickness, 8);

                    //imshow( "Road facing camera", currImage_c );
                    imshow( "Trajectory on x/y axis", traj);
                    imshow( "Trajectory on x/z axis", traj2);

                    waitKey(1);

                    geometry_msgs::PoseStamped error;
                    error.header.stamp =  ros::Time::now();
                    error.pose.position.x = (float)sqrt(pow((latest_ground_truth.pose.position.x - t_f.at<double>(0)), 2) + pow((latest_ground_truth.pose.position.y - t_f.at<double>(1)), 2) + pow((latest_ground_truth.pose.position.z - t_f.at<double>(2)),2));
                    error_pub.publish(error);


                    //Reset counter
                    counter = 0;

                    // Determine time that has passed since last call
                    ros::Time curr_call = ros::Time::now();
                    ros::Duration temp_dur = curr_call - last_call;
                    last_call = curr_call;
                    float elapsed_time = temp_dur.toSec();

                    if(elapsed_time < min_elapsed_time){
                        min_elapsed_time = elapsed_time;
                    }
                    if(elapsed_time > max_elapsed_time){
                        max_elapsed_time = elapsed_time;
                    }
                    cout << "Min execution time = "<<min_elapsed_time << "s, Max execution time = "<<max_elapsed_time<<"s"<<endl;
                    cout << "Min number of features = "<<min_num_features << ", Max number of features = "<<max_num_features<<endl;
                    //cout << "Execution time estimated to " << elapsed_time << "s" << endl;

                    ros::spinOnce();
                    loop_rate.sleep();
                }

                //Recovery strategy if not enough features are detected
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

                //Publish IMU data to the EKF
                sensor_msgs::Imu latest_imu_data = imu_data;
                latest_imu_data.orientation_covariance[0] = cov_imu_x;
                latest_imu_data.orientation_covariance[4] = cov_imu_y;
                latest_imu_data.orientation_covariance[8] = cov_imu_z;
                latest_imu_data.header.frame_id = tf_frame;
                latest_imu_data.header.stamp = ros::Time::now();
                ekf_imu_pub.publish(latest_imu_data);

                //

            }
            ros::spinOnce();
            loop_rate.sleep();
        }//End while loop
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

    //IMU Callback
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg_in)
    {
        //Store latest imu information with corresponding time stamp
        time_imu = msg_in->header.stamp;
        imu_data.orientation = msg_in->orientation;
        imu_data.angular_velocity = msg_in->angular_velocity;
        imu_data.linear_acceleration = msg_in->linear_acceleration;
        return;
    }

    //EKF odometry callback
    void ekfodoCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_in)
    {      
        double loc_rot_x,loc_rot_y, loc_rot_z, loc_rot_w;
        loc_rot_x = msg_in->pose.pose.orientation.x;
        loc_rot_y = msg_in->pose.pose.orientation.y;
        loc_rot_z = msg_in->pose.pose.orientation.z;
        loc_rot_w = msg_in->pose.pose.orientation.w;
        t_f.at<double>(0,0) = msg_in->pose.pose.position.x;
        t_f.at<double>(1,0) = msg_in->pose.pose.position.y;
        t_f.at<double>(2,0) = msg_in->pose.pose.position.z;
        //Quaternion to rotational matrix formulas can be found at http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
        R_f.at<double>(0,0) = 1 - 2*loc_rot_y*loc_rot_y - 2*loc_rot_z*loc_rot_z;
        R_f.at<double>(0,1) = 2*loc_rot_x*loc_rot_y - 2*loc_rot_z*loc_rot_w;
        R_f.at<double>(0,2) = 2*loc_rot_x*loc_rot_z + 2*loc_rot_y*loc_rot_w;
        R_f.at<double>(1,0) = 2*loc_rot_x*loc_rot_y + 2*loc_rot_z*loc_rot_w;
        R_f.at<double>(1,1) = 1 - 2*loc_rot_x*loc_rot_x - 2*loc_rot_z*loc_rot_z;
        R_f.at<double>(1,2) = 2*loc_rot_y*loc_rot_z - 2*loc_rot_x*loc_rot_w;
        R_f.at<double>(2,0) = 2*loc_rot_x*loc_rot_z - 2*loc_rot_y*loc_rot_w;
        R_f.at<double>(2,1) = 2*loc_rot_y*loc_rot_z + 2*loc_rot_x*loc_rot_w;
        R_f.at<double>(2,2) = 1 - 2*loc_rot_x*loc_rot_x - 2*loc_rot_y*loc_rot_y;
    }

protected:

    // Subscriber and Publisher
    ros::Subscriber camera_sub;
    ros::Subscriber truth_odometry_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber ekf_odo_sub;
    ros::Publisher ekf_vo_pub;
    ros::Publisher ekf_imu_pub;
    ros::Publisher error_pub;

    //Variable for initialization
    int init_counter;
    bool init_flag;
    Mat img_1_c, img_2_c;
    double rot_x, rot_y, rot_z, rot_w;
    double cov_x, cov_y, cov_z, cov_rot_x, cov_rot_y, cov_rot_z;
    double cov_imu_x, cov_imu_y, cov_imu_z;

    //Parameters for the camera
    double scale;
    double focal;
    double xpp;
    double ypp;
    string tf_frame;

    //Parameters for the topics
    string camera_topic_sub, imu_topic_sub, groundtruth_topic_sub;

    //Variables for position and rotation
    double start_x;
    double start_y;
    double start_z;
    double start_rot_x;
    double start_rot_y;
    double start_rot_z;

    //Parameters for min/max computation time
    float min_elapsed_time;
    double min_num_features;
    float max_elapsed_time;
    double max_num_features;

    //Parameters for the algorithm
    Mat R_f, t_f;                   //The final rotation and tranlation vectors
    double max_frame;
    double min_feature;
    double min_feature_recalculate;

    //Parameters for RANSAC
    double ransac_prob;
    double ransac_threshold;

    //Parameters for path to files
    string _groud_truth_file;
    string _images_path;

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
