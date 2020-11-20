/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <mutex>
#include <thread>
#include <condition_variable>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"
#include "../../../include/Converter.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM) : mpSLAM(pSLAM), updated(false) {}
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void GrabCompressedRGBD(sensor_msgs::CompressedImageConstPtr msgRGB, sensor_msgs::CompressedImageConstPtr msgD);
    void getImages(cv::Mat& im, cv::Mat& imD, double& timestamp);

    ORB_SLAM2::System* mpSLAM;
    bool updated;

private:
    cv::Mat imRGB, imDepth;
    double ts;
    std::mutex imgMutex;
};

class ROSPublisher
{
public:
    const std::string base_frame;
    const std::string camera_frame;
    bool dataReady;

    ROSPublisher(ros::NodeHandle n) :
        base_frame("world"), camera_frame("camera")
    {            
        pub_odom = n.advertise<nav_msgs::Odometry>(odom_topic, 1);
        pub_traj = n.advertise<visualization_msgs::Marker>(traj_topic, 100);
        pub_pc = n.advertise<sensor_msgs::PointCloud2>(pc_topic, 10);
        color.a = 1.0;
        color.r = 255.0/255.0;
        color.g = 192.0/255.0;
        color.b = 203.0/255.0;
        prevPt.x = 0;
        prevPt.y = 0;
        prevPt.z = 0;
        markerCount = 0;
        dataReady = false;
    }

    void startThread()
    {
        pubThread = std::thread(&ROSPublisher::publishAsync, this);
    }

    void publish(const cv::Mat& imRGB, const cv::Mat& imDepth, const cv::Mat& T)
    {
        ros::Time time_now = ros::Time::now();

        cv::Mat R = T.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat t = -R * T.rowRange(0, 3).col(3);
        std::vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);

        tf::Transform new_transform;
        new_transform.setOrigin(tf::Vector3(t.at<float>(0, 2), t.at<float>(0, 0), t.at<float>(0, 1)));
        tf::Quaternion tf_quaternion(q[2], q[0], q[1], q[3]); // z, x, y, w
        new_transform.setRotation(tf_quaternion);

        tf::StampedTransform stamped_transform = tf::StampedTransform(new_transform, time_now, base_frame, camera_frame);
        br.sendTransform(stamped_transform);

        geometry_msgs::Pose pose_msg;
        tf::poseTFToMsg(new_transform, pose_msg);

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = time_now;
        odom_msg.header.frame_id = base_frame;
        odom_msg.child_frame_id = camera_frame;
        odom_msg.pose.pose = pose_msg;

        // Prepare point cloud
        double tick = (double)cv::getTickCount();

        const float fx = 615.268371582031, fy = 615.40625, cx = 323.562591552734, cy = 241.798583984375;
        cv::Mat imDepth32F, imDDown, imRGBDown;
        imDepth.convertTo(imDepth32F, CV_32F, 1.0f/1000.0f);

        pcl::PointCloud<pcl::PointXYZRGB> pc;
        sensor_msgs::PointCloud2 pc_msg;

        for (int v = 0; v < imDepth32F.rows; v++)
        {
            for (int u = 0; u < imDepth32F.cols; u++)
            {
                //Back project 2D pixel to 3D point
                // TODO:
                float X, Y, Z;
                std::uint8_t r, g, b;

                Z = imDepth32F.at<float>(v, u); 
                X = (u - cx) * Z / fx;
                Y = (v - cy) * Z / fy;
                r = imRGB.at<cv::Vec3b>(v, u)[0];
                g = imRGB.at<cv::Vec3b>(v, u)[1];
                b = imRGB.at<cv::Vec3b>(v, u)[2];

                pcl::PointXYZRGB point(r, g, b);
                point.x = Z; point.y = X; point.z = Y;
                pc.push_back(point);                
            }
        }
        pcl::toROSMsg(pc, pc_msg);
        pc_msg.header.stamp = time_now;
        pc_msg.header.frame_id = camera_frame;

        pub_odom.publish(odom_msg);
        pub_pc.publish(pc_msg);

        ROS_INFO("Elapsed time: %.5f", ((double)cv::getTickCount() - tick)/cv::getTickFrequency());
    }

    void publishAsync()
    {
        while (ros::ok())
        {
            std::unique_lock<std::mutex> lock(dataMutex);
            if (!condVar.wait_for(lock, std::chrono::milliseconds(500), std::bind(&ROSPublisher::isDataReady, this)))
            {
                if (!ros::ok()) break;
                else continue;
            }

            ros::Time time_now = ros::Time::now();

            cv::Mat R = Tcw.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat t = -R * Tcw.rowRange(0, 3).col(3);
            std::vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);

            tf::Transform new_transform;
            new_transform.setOrigin(tf::Vector3(t.at<float>(0, 2), -t.at<float>(0, 0), -t.at<float>(0, 1)));
            tf::Quaternion tf_quaternion(q[2], q[0], q[1], q[3]); // z, x, y, w
            new_transform.setRotation(tf_quaternion);

            tf::StampedTransform stamped_transform = tf::StampedTransform(new_transform, time_now, base_frame, camera_frame);
            br.sendTransform(stamped_transform);

            geometry_msgs::Pose pose_msg;
            tf::poseTFToMsg(new_transform, pose_msg);

            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = time_now;
            odom_msg.header.frame_id = base_frame;
            odom_msg.child_frame_id = camera_frame;
            odom_msg.pose.pose = pose_msg;

            // Prepare trajectory marker
            visualization_msgs::Marker traj_msg;
            traj_msg.header.frame_id = base_frame;
            traj_msg.ns = "trajectory";
            traj_msg.id = markerCount++;
            traj_msg.type = visualization_msgs::Marker::LINE_STRIP;
            traj_msg.action = visualization_msgs::Marker::ADD;
            traj_msg.scale.x = 0.1;
            traj_msg.color = color;
            traj_msg.lifetime = ros::Duration(0.0);
            traj_msg.header.stamp = time_now;
            traj_msg.pose.orientation.w = 1.0;

            traj_msg.points.push_back(prevPt);
            traj_msg.points.push_back(pose_msg.position);

            // Prepare point cloud

            const float fx = 615.268371582031, fy = 615.40625, cx = 323.562591552734, cy = 241.798583984375;
            // cv::resize(imDepth32F, imDDown, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
            // cv::resize(imRGB, imRGBDown, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

            pcl::PointCloud<pcl::PointXYZRGB> pc;
            sensor_msgs::PointCloud2 pc_msg;

            for (int v = 0; v < imDepth32F.rows; v++)
            {
                for (int u = 0; u < imDepth32F.cols; u++)
                {
                    //Back project 2D pixel to 3D point
                    // TODO:
                    float X, Y, Z;
                    std::uint8_t r, g, b;

                    Z = imDepth32F.at<float>(v, u); 
                    // Apply threshold filter for depth points above 5 meters
                    if (Z > 5000.0f) continue;
                    X = (u - cx) * Z / fx;
                    Y = (v - cy) * Z / fy;
                    r = imRGB_.at<cv::Vec3b>(v, u)[0];
                    g = imRGB_.at<cv::Vec3b>(v, u)[1];
                    b = imRGB_.at<cv::Vec3b>(v, u)[2];

                    pcl::PointXYZRGB point(r, g, b);
                    // Flip the point cloud and align it to the same direction as the camera
                    point.x = Z; point.y = X; point.z = -Y;
                    pc.push_back(point);                
                }
            }
            pcl::toROSMsg(pc, pc_msg);
            pc_msg.header.stamp = time_now;
            pc_msg.header.frame_id = camera_frame;

            pub_odom.publish(odom_msg);
            pub_traj.publish(traj_msg);
            pub_pc.publish(pc_msg);

            prevPt = pose_msg.position;
            dataReady = false;
       }
    }

    void getData(cv::Mat& im, cv::Mat& imD, cv::Mat& pose)
    {
        std::lock_guard<std::mutex> guard(dataMutex);
        imD.convertTo(imDepth32F, CV_32F, 1.0f/1000.0f);
        // im.copyTo(imRGB_);
        cv::cvtColor(im, imRGB_, CV_GRAY2RGB);
        pose.copyTo(Tcw);    
        dataReady = true;
        condVar.notify_one();
    }

    bool isDataReady()
    {
        return dataReady;
    }

private:
    const std::string odom_topic = "/RGBD/odom";
    const std::string traj_topic = "/RGBD/trajectory";
    const std::string pc_topic = "/RGBD/pointcloud";
    ros::Publisher pub_odom;
    ros::Publisher pub_traj;
    ros::Publisher pub_pc;
    tf::TransformBroadcaster br;
    geometry_msgs::Point prevPt;
    std_msgs::ColorRGBA color;
    int markerCount;

    cv::Mat imDepth32F;
    cv::Mat imRGB_;
    cv::Mat Tcw;

    std::thread pubThread;
    std::mutex dataMutex;
    std::condition_variable condVar;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    ROSPublisher publisher(nh);
    publisher.startThread();

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    cv::Mat imRGB, imDepth;
    double timestamp;

    while (ros::ok())
    {
        if (igb.updated)
        {                                
            igb.getImages(imRGB, imDepth, timestamp);
            // cv::Mat pose = SLAM.TrackRGBD(imRGB, imDepth, timestamp);
            cv::Mat imDepthFilt;
            cv::GaussianBlur(imDepth, imDepthFilt, cv::Size(9, 9), 0.80, 0.0);
            cv::Mat pose = SLAM.TrackRGBD(imRGB, imDepthFilt, timestamp);

            int trackState = SLAM.GetTrackingState();
            switch (trackState)
            {                    
                case ORB_SLAM2::Tracking::SYSTEM_NOT_READY:
                    ROS_INFO("Track status: NOT READY");
                    break;
                case ORB_SLAM2::Tracking::NO_IMAGES_YET:
                    ROS_INFO("Track status: NO IMAGES YET");
                    break;
                case ORB_SLAM2::Tracking::NOT_INITIALIZED:
                    ROS_INFO("Track status: NOT INITIALIZED");
                    break;
                case ORB_SLAM2::Tracking::OK:
                    ROS_INFO("Track status: OK");
                    // publisher.publish(imRGB, imDepth, pose);
                    publisher.getData(imRGB, imDepth, pose);
                    break;
                case ORB_SLAM2::Tracking::LOST:
                    ROS_INFO("Track status: LOST");
                    break;
            }
        }
        ros::spinOnce();
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    {
    std::unique_lock<std::mutex> lock(imgMutex);
    cv_ptrRGB->image.copyTo(imRGB);
    cv_ptrD->image.copyTo(imDepth);
    ts = cv_ptrRGB->header.stamp.toSec();
    updated = true;
    }
}

void ImageGrabber::GrabCompressedRGBD(sensor_msgs::CompressedImageConstPtr msgRGB, sensor_msgs::CompressedImageConstPtr msgD)
{
    std::vector<uint8_t> rgb = msgRGB->data; 
    std::vector<uint8_t> depth = msgD->data;
    cv::Mat im1 = cv::imdecode(rgb, cv::IMREAD_UNCHANGED); 
    cv::Mat im2 = cv::imdecode(depth, cv::IMREAD_UNCHANGED);

    {
    std::unique_lock<std::mutex> lock(imgMutex);
    im1.copyTo(imRGB);
    im2.copyTo(imDepth);
    ts = msgRGB->header.stamp.toSec();
    updated = true;
    }
}

void ImageGrabber::getImages(cv::Mat& im, cv::Mat& imD, double& timestamp)
{
    std::unique_lock<std::mutex> lock(imgMutex);
    imRGB.copyTo(im);
    imDepth.copyTo(imD);
    timestamp = ts;
    updated = false;
}
