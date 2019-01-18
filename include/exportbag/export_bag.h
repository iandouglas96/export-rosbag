#ifndef EXPORT_BAG_H
#define EXPORT_BAG_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <string>
#include <sys/stat.h> //Filesystem unix lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <flex_sync/sync.h>

namespace exportbag {
  class ExportBag {
  public:
    ExportBag(const ros::NodeHandle &nh);
    ~ExportBag();

    bool initialize();
  private:
    void processBag();
    void createDirectories();
    void processSyncImages(const std::vector<sensor_msgs::CompressedImageConstPtr> &img_vec);
    void processImage(const std::string &topic, const sensor_msgs::CompressedImageConstPtr &img);
    void processPointCloud(const std::string &topic, const sensor_msgs::PointCloud2ConstPtr &pc);
    void processImu(const sensor_msgs::ImuConstPtr &imu);

    ros::NodeHandle nh_;

    std::map<std::string, int> topicCounts_;
    std::map<std::string, std::string> topicNames_;

    sensor_msgs::PointCloud2ConstPtr lastPointCloud_;

    //params
    std::vector<std::string> imageTopics_;
    std::vector<std::string> pointCloudTopics_;
    std::string imuTopic_;
    std::string bagFile_;
    std::string outputDir_;
    bool syncImages_;
    bool syncLidarWithImages_;

    bool useCLAHE_;
    cv::Ptr<cv::CLAHE> clahe_;
  };
}

#endif //EXPORT_BAG_H
