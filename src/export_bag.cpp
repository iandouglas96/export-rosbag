/*
 * rosbag exporter
 * Ian Miller 2018, iandm@seas.upenn.edu
 * rosbag import code modified from Bernd Pfrommer 
 *    (https://github.com/berndpfrommer/tagslam)
 */

#include "exportbag/export_bag.h"

namespace exportbag {
  ExportBag::ExportBag(const ros::NodeHandle &nh) {
    nh_ = nh;
  } 

  ExportBag::~ExportBag() {
  }

  bool ExportBag::initialize() {
    nh_.getParam("image_topics", imageTopics_);
    nh_.getParam("pointcloud_topics", pointcloudTopics_);
    nh_.getParam("imu_topic", imuTopic_);

    if (!nh_.getParam("bag_file", bagFile_)) {
      ROS_ERROR("Must specify bag to process (bag_file)!");
      return false;
    }

    if (!nh_.getParam("output_dir", outputDir_)) {
      ROS_ERROR("Must specify output directory (output_dir)!");
      return false;
    }

    processBag();    
  }

  void ExportBag::processBag() {
    rosbag::Bag bag;
    bag.open(bagFile_, rosbag::bagmode::Read);
    ROS_INFO_STREAM("loading bag file: " << bagFile_);

    std::vector<std::string> allTopics = imageTopics_;
    allTopics.insert(allTopics.end(), pointcloudTopics_.begin(), pointcloudTopics_.end());
    allTopics.insert(allTopics.end(), imuTopic_);

    createDirectories();

    //Init counts to 0
    for (auto &topic : allTopics) {
      topicCounts_.insert(topicCounts_.begin(), std::pair<std::string, int>(topic, 0));
    }

    rosbag::View view(bag, rosbag::TopicQuery(allTopics), rosbag::View(bag).getBeginTime(),
        ros::TIME_MAX);

    //Go through bag, 1 topic at a time
    for (const rosbag::MessageInstance &m: view) {
      //Determine message type, process accordingly
      if (m.getDataType() == "sensor_msgs/CompressedImage") {
        const sensor_msgs::CompressedImageConstPtr img = 
          m.instantiate<sensor_msgs::CompressedImage>();
        processImage(m.getTopic(), img);
      }
    } 
  }

  void ExportBag::createDirectories() {
    //Name folders
    int n = 0;
    std::string dir;
    for (auto &topic : imageTopics_) {
      dir = "image"+std::to_string(n);
      topicNames_.insert(topicNames_.begin(), std::pair<std::string, std::string>(
            topic, dir));
      mkdir((outputDir_+"/"+dir).c_str(), 0777);
      mkdir((outputDir_+"/"+dir+"/data").c_str(), 0777);
      n++;
    }
  }

  void ExportBag::processImage(const std::string &topic,
      const sensor_msgs::CompressedImageConstPtr &img) {
    cv::Mat origImage = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8)->image; 
    cv::Mat colorImage;
    //Debayer
    cv::cvtColor(origImage, colorImage, cv::COLOR_BayerBG2BGR);

    int count = topicCounts_.at(topic);
    std::ostringstream stream;
    stream << outputDir_ << "/";
    stream << topicNames_.at(topic) << "/data/";
    stream << std::setfill('0') << std::setw(8) << count << ".jpg";

    ROS_INFO_STREAM(stream.str());

    cv::imwrite(stream.str(), colorImage);

    topicCounts_.at(topic) = count + 1;
  }
}
