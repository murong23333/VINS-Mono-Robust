#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

ros::Publisher pub_depth_img;

// Extrinsics Lidar -> Camera
// R_cl, t_cl calculated previously
Matrix3d R_cl;
Vector3d t_cl;

// Camera Intrinsics (Hardcoded from camera_right.yaml for verification)
double fx = 431.3364265799752;
double fy = 432.7527965378035;
double cx = 354.8956286992647;
double cy = 232.5508916495161;
// Distortion
double k1 = -0.300267420221178;
double k2 = 0.090544063693053;
double p1 = 3.330220891093334e-05;
double p2 = 8.989607188457415e-05;

void sync_callback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    int valid_points = 0;
    for (const auto &pt : cloud.points)
    {
        // Transform to Camera Frame
        Vector3d P_l(pt.x, pt.y, pt.z);
        Vector3d P_c = R_cl * P_l + t_cl;

        if (P_c.z() < 0.5) continue; // Behind camera or too close

        // Project to Image Plane (Pinhole + Distortion)
        double X = P_c.x();
        double Y = P_c.y();
        double Z = P_c.z();
        
        double x = X / Z;
        double y = Y / Z;
        
        // Distortion
        double r2 = x*x + y*y;
        double r4 = r2*r2;
        double x_dist = x * (1 + k1*r2 + k2*r4) + 2*p1*x*y + p2*(r2 + 2*x*x);
        double y_dist = y * (1 + k1*r2 + k2*r4) + p1*(r2 + 2*y*y) + 2*p2*x*y;
        
        double u = fx * x_dist + cx;
        double v = fy * y_dist + cy;

        if (u >= 0 && u < 752 && v >= 0 && v < 480)
        {
            // Color based on distance (Turbo/Jet like)
            double depth = Z;
            int r = std::min(255.0, std::max(0.0, 255 - (depth - 2.0) * 30.0));
            int g = std::min(255.0, std::max(0.0, (depth - 2.0) * 30.0));
            cv::circle(cv_ptr->image, cv::Point(u, v), 1, cv::Scalar(0, g, r), -1);
            valid_points++;
        }
    }

    // ROS_INFO("Projected %d points", valid_points);
    sensor_msgs::ImagePtr pub_msg = cv_ptr->toImageMsg();
    pub_depth_img.publish(pub_msg);

    static int save_once = 0;
    if (save_once == 0 && valid_points > 500) {
        cv::imwrite("/output/depth_verify.png", cv_ptr->image);
        ROS_INFO("Saved verification image to /output/depth_verify.png with %d points", valid_points);
        save_once = 1;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_verification_node");
    ros::NodeHandle n;

    // Initialize Extrinsics
    R_cl << -0.01916508, 0.9997437, -0.01205075,
            -0.01496218, 0.01176483, 0.99981885,
             0.99970438, 0.01934191, 0.01473287;
    t_cl << -0.13417409, 0.03958218, -0.05719056;

    pub_depth_img = n.advertise<sensor_msgs::Image>("/depth_verification_image", 10);

    // Sync
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> img_sub(n, "/right/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, "/os1_cloud_node1/points", 100);

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_sub, cloud_sub);
    sync.registerCallback(boost::bind(&sync_callback, _1, _2));

    ROS_INFO("Depth Verification Node Started");
    ros::spin();

    return 0;
}
