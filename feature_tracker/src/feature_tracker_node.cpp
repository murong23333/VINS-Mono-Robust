#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <mutex>
#include <queue>

#include "feature_tracker.h"

struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;          // The channel index
    uint16_t noise;
    uint32_t range;         // The distance measurement
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint32_t, t, t)
                                  (uint16_t, reflectivity, reflectivity)
                                  (uint8_t,  ring, ring)
                                  (uint16_t, noise, noise)
                                  (uint32_t, range, range))

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

// Lidar-Camera Extrinsics
Eigen::Matrix3d R_cl;
Eigen::Vector3d t_cl;
// Depth map for current frame
cv::Mat current_depth_map;

// IMU Buffer for Deskewing
queue<sensor_msgs::ImuConstPtr> imu_buf;
std::mutex m_buf;

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
}


// Robust Depth Lookup
float getDepthRough(const cv::Mat &depth_map, int u, int v) {
    const int R = 5; // 5 pixel radius
    std::vector<float> valid_depths;
    valid_depths.reserve((2*R+1)*(2*R+1));
    
    for (int dy = -R; dy <= R; dy++) {
        for (int dx = -R; dx <= R; dx++) {
            int x = u + dx;
            int y = v + dy;
            if (x < 0 || x >= depth_map.cols || y < 0 || y >= depth_map.rows) continue;
            float d = depth_map.at<float>(y, x);
            if (d > 0.1 && d < 100.0) {
                valid_depths.push_back(d);
            }
        }
    }
    
    if (valid_depths.size() < 3) return -1.0;
    
    float sum = 0;
    for (float d : valid_depths) sum += d;
    float mean = sum / valid_depths.size();
    
    float sq_sum = 0;
    for (float d : valid_depths) sq_sum += (d - mean) * (d - mean);
    float std_dev = std::sqrt(sq_sum / valid_depths.size());
    
    // If standard deviation is high (> 0.5m), it's an edge/outlier
    if (std_dev > 0.5) return -1.0;
    
    return mean;
}

// void img_callback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::PointCloud2ConstPtr &lidar_msg)
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // [Removed Lidar Deskewing and Depth Processing logic for pure VINS-Mono]
    // current_depth_map = cv::Mat::zeros(480, 752, CV_32F); 
    
    // --- DESKEWING REMOVED ---


    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = img_msg->header.stamp.toSec();
    // frequency control
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec());
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                    
                    // [Removed Depth Lookup]
                    /*
                    // Lookup depth in current_depth_map
                    double depth = -1;
                    int u = round(cur_pts[j].x); // This is NOT pixel coordinate, cur_pts is pixel
                    int v = round(cur_pts[j].y);
                    if (u >= 0 && u < 752 && v >= 0 && v < 480 && !current_depth_map.empty())
                    {
                         depth = getDepthRough(current_depth_map, u, v);
                    }
                    if (depth > 0) {
                        // ROS_DEBUG("Feature %d has depth %f", p_id, depth);
                    }
                    if (depth > 0) {
                         // Use depth if available, otherwise -1
                    }
                    */
                }
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        
        // Add Depth Channel
        /*
        sensor_msgs::ChannelFloat32 depth_of_point;
        depth_of_point.name = "depth";
        for (int i = 0; i < NUM_OF_CAM; i++) { // Re-iterate to fill depth for all points
             auto &cur_pts = trackerData[i].cur_pts;
             auto &ids = trackerData[i].ids;
             for (unsigned int j = 0; j < ids.size(); j++) {
                 if (trackerData[i].track_cnt[j] > 1) {
                     double d = -1;
                     int u = round(cur_pts[j].x);
                     int v = round(cur_pts[j].y);
                     if (u >= 0 && u < 752 && v >= 0 && v < 480 && !current_depth_map.empty())
                     {
                          d = getDepthRough(current_depth_map, u, v);
                     }
                     depth_of_point.values.push_back(d);
                 }
             }
        }
        feature_points->channels.push_back(depth_of_point);
        */
        
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img.publish(feature_points);

        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match.publish(ptr->toImageMsg());
        }
    }
    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);
    
    /*
    // Initialize Extrinsics (Lidar -> Camera)
    R_cl << -0.01916508, 0.9997437, -0.01205075,
            -0.01496218, 0.01176483, 0.99981885,
             0.99970438, 0.01934191, 0.01473287;
    t_cl << -0.13417409, 0.03958218, -0.05719056;

    // Sync Subscriber
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> img_sub(n, IMAGE_TOPIC, 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, "/os1_cloud_node1/points", 100);
    
    // Use large queue size and tolerance for robust sync
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), img_sub, cloud_sub);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));

    // Subscribe to Lidar IMU for deskewing
    ros::Subscriber sub_imu = n.subscribe("/os1_cloud_node1/imu", 1000, imu_callback);
    */

    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?