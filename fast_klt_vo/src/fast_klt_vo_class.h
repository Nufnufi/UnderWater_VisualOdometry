
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "vo_features.h"

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

        //Start position and rotation
        nh.param<double>("/odometry/start_x", this->x, 0.2);
        nh.param<double>("/odometry/start_y", this->y, 0.2);
        nh.param<double>("/odometry/start_z", this->z, 0.2);
        nh.param<double>("/odometry/start_rot_x", this->rot_x, M_PI/2.0);
        nh.param<double>("/odometry/start_rot_y", this->rot_y, M_PI/2.0);
        nh.param<double>("/odometry/start_rot_z", this->rot_z, M_PI/2.0);

        //Parameters for camera
        nh.param<double>("/camera/focal", this->focal, 405.6385);   //718
        nh.param<double>("/camera/xpp", this->xpp, 189.9054);       //607.1928
        nh.param<double>("/camera/ypp", this->ypp, 139.915);               //185.2157
        nh.param<double>("/camera/scale", this->scale, 1);
        cout<<"Used camera has parameters"<<endl;

        //Parameters for KLT algorithm
        nh.param<double>("/odometry/max_frame", this->max_frame, 7000);                             //Number of input images
        nh.param<double>("/odometry/min_feature_recalculate", this->min_feature_recalculate, 14);   //Set min number of feature for redetection
        nh.param<double>("/odometry/min_feature", this->min_feature, 6);                            //Set min number of features necessary to compute the essential matrix

        //Parameters for RANSAC
        nh.param<double>("/ransac/probability", this->ransac_prob, 0.999);
        nh.param<double>("/ransac/threshold", this->ransac_threshold, 1.0);

        // Read ROS parameter for path to image
        nh.param<string>("/image/groundtruth_file_path", _groud_truth_file, "/home/maxime/Datasets/KITTI_VO/ground_poses/00.txt");
        nh.param<string>("/image/path_to_images", _images_path, "/home/maxime/Datasets/uwcaves/frames/frame_%06d.png");

        // introduce publishers
        visual_odometry_pub = nh.advertise<geometry_msgs::PoseStamped>("/auv/visualodometry", 10);

        // introduce subscribers
        camera_sub = nh.subscribe("/camera/image", 10, &fast_klt_vo::cameraCallback, this);

        ros::Time::init();
        ros::Duration(0.1).sleep();
    }

    //////////////////////////////////////////////
    // pseudo function for handling odometry
    void start()
    {
        ros::Rate loop_rate(100);

        while (ros::ok())
        {         
            Mat img_1, img_2;   //Matrices to store the images
            Mat R_f, t_f;       //The final rotation and tranlation vectors

            ofstream myfile;
            myfile.open ("results1_1.txt"); //Create results document

            //Load the first images path
            char filename1[200];
            char filename2[200];
            const char * path = _images_path.c_str(); //Convert string to constant chart
            sprintf(filename1, path, 0);
            sprintf(filename2, path, 1);

            char text[100];
            int fontFace = FONT_HERSHEY_PLAIN;
            double fontScale = 1;
            int thickness = 1;
            cv::Point textOrg(10, 50);

            //Read the first two frames from the dataset (Color)
            Mat img_1_c = imread(filename1);
            Mat img_2_c = imread(filename2);

            if ( !img_1_c.data || !img_2_c.data ) {
              std::cout<< " --(!) Error reading images " << std::endl; return;
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

            Mat prevImage = img_2; //Keep previous image
            Mat currImage;
            vector<Point2f> prevFeatures = points2; //Keep features from previous image
            vector<Point2f> currFeatures, copy_prevFeatures;
            char filename[100];

            //Initialize final roation and translation matrices
            R_f = R.clone();
            t_f = t.clone();

            //Initialize display windows
            namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
            namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.
            Mat traj = Mat::zeros(600, 600, CV_8UC3);
            int counter;
            counter = 0;

            //Loop for all images
            for(int numFrame=2 ; numFrame < max_frame; numFrame++){
                last_call = ros::Time::now();   //Start timmer
                sprintf(filename, path, numFrame);
                printf("/home/maxime/Datasets/uwcaves/frames/frame_%06d.png", numFrame);

                //Load current image (color)
                Mat currImage_c = imread(filename);
                cvtColor(currImage_c, currImage, COLOR_BGR2GRAY); //Convert to greyscale
                vector<uchar> status;

                //Feature detection and tracking
                copy_prevFeatures = prevFeatures;
                featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
                ros::spinOnce();
                loop_rate.sleep();

                if(currFeatures.size() > min_feature){   //If enough features have been detected and tracked, compute the essential matrix
                    E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, ransac_prob, ransac_threshold, mask);
                    //cout << currFeatures.size()<<"  "<< prevFeatures.size()<<"E size = "<< E.size()<< " E = "<< E<< " mask"<< mask.size() <<endl;
                    recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

                    Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);
                    for(int i=0;i<prevFeatures.size();i++)	{   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
                              prevPts.at<double>(0,i) = prevFeatures.at(i).x;
                              prevPts.at<double>(1,i) = prevFeatures.at(i).y;

                              currPts.at<double>(0,i) = currFeatures.at(i).x;
                              currPts.at<double>(1,i) = currFeatures.at(i).y;
                    }
                   // cout << "t = " << t << endl;
                    scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));
                    //cout << "Scale is " << scale << endl;

                    //Update rotation and translation matrices
                    if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
                        t_f = t_f + scale*(R_f*t);
                        R_f = R*R_f;
                    }
                    else{
                        cout << "scale below 0.1, or incorrect translation" << endl;
                        //cout<<"tf = "<<t_f <<" Rf = "<<R_f<<"E = "<<E<<endl;
                     }

                    //Update previous image and features
                    prevImage = currImage.clone();
                    prevFeatures = currFeatures;
                    ros::spinOnce();
                    loop_rate.sleep();

                    //Display trajectory
                    int x = int(t_f.at<double>(0)) + 300;
                    int y = int(t_f.at<double>(2)) + 350;
                    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);
                    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
                    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
                    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
                    imshow( "Road facing camera", currImage_c );
                    imshow( "Trajectory", traj );
                    waitKey(1);

                    //Reset counter
                    counter = 0;

                    // Determine time that has passed since last call
                    ros::Time curr_call = ros::Time::now();
                    ros::Duration temp_dur = curr_call - last_call;
                    last_call = curr_call;
                    float elapsed_time = temp_dur.toSec();

                    cout << "Execution time estimated to " << elapsed_time << "s" << endl;
                    //cout << R_f << endl;
                    //cout << t_f << endl;

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
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    double getAbsoluteScale(int frame_id, int sequence_id, double z_cal){ //Absolute scale from kitti dataset is on a ground truth

      string line;
      int i = 0;
      ifstream myfile (_groud_truth_file.c_str()); //Load ground truth
      double x =0, y=0, z = 0;
      double x_prev, y_prev, z_prev;
      if (myfile.is_open())
      {
        while (( getline (myfile,line) ) && (i<=frame_id))
        {
          z_prev = z;
          x_prev = x;
          y_prev = y;
          std::istringstream in(line);
          //cout << line << '\n';
          for (int j=0; j<12; j++)  {
            in >> z ;
            if (j==7) y=z;
            if (j==3)  x=z;
          }

          i++;
        }
        myfile.close();
      }

      else {
        cout << "Unable to open file";
        return 0;
      }

      return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

    }

    //////////////////////////////////////////////
    // callback function for the estimated linear and angular velocities
    void cameraCallback(const sensor_msgs::Image::ConstPtr& msg_in)
    {
       /*DO conversion from sensor_msgs/Image message to Mat for the latest image*/
        return;
    }



protected:

    // Subscriber and Publisher
    ros::Subscriber camera_sub;
    ros::Publisher visual_odometry_pub;

    //Parameters for the camera
    double scale;
    double focal;
    double xpp;
    double ypp;

    //Variables for position and rotation
    double x;
    double y;
    double z;
    double rot_x;
    double rot_y;
    double rot_z;

    //Parameters for the algorithm
    double max_frame;
    double min_feature;
    double min_feature_recalculate;

    //Parameters for RANSAC
    double ransac_prob;
    double ransac_threshold;

    //Parameters for path to files
    string _groud_truth_file;
    string _images_path;

    //Variables to store latest image received
    Mat latest_image;

    ros::Time last_call;
};



