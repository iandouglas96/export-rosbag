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
    nh_.getParam("pointcloud_topics", pointCloudTopics_);
    nh_.getParam("imu_topic", imuTopic_);
    nh_.getParam("sync_images", syncImages_);
    nh_.getParam("sync_lidar_with_images", syncLidarWithImages_);
    
    nh_.getParam("use_clahe", useCLAHE_);
    float claheClipLimit;
    float claheTileCount;
    nh_.getParam("clahe_clip_limit", claheClipLimit);
    nh_.getParam("clahe_tile_count", claheTileCount);
    
    nh_.getParam("rectify", rectify_);
    if (rectify_) {
      std::string rectifyPath;
      nh_.getParam("rectify_path", rectifyPath);
      initRectifyMaps(rectifyPath);
    }

    clahe_ = cv::createCLAHE();
    clahe_->setClipLimit(claheClipLimit);
    clahe_->setTilesGridSize(cv::Size(claheTileCount, claheTileCount));

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
    
    createDirectories();

    std::vector<std::string> allTopics = imageTopics_;
    allTopics.insert(allTopics.end(), pointCloudTopics_.begin(), pointCloudTopics_.end());
    allTopics.insert(allTopics.end(), imuTopic_);
    
    //Init counts to 0
    for (auto &topic : allTopics) {
      topicCounts_.insert(topicCounts_.begin(), std::pair<std::string, int>(topic, 0));
    }
    
    //setup sync
    std::vector<std::vector<std::string>> sync_topics;
    sync_topics.push_back(imageTopics_);
    flex_sync::Sync<sensor_msgs::CompressedImage> image_sync(
         sync_topics, std::bind(&ExportBag::processSyncImages, this, std::placeholders::_1));
    
    rosbag::View view(bag, rosbag::TopicQuery(allTopics), rosbag::View(bag).getBeginTime(),
        ros::TIME_MAX);

    //Go through bag, 1 topic at a time
    for (const rosbag::MessageInstance &m: view) {
      //Determine message type, process accordingly
      if (m.getDataType() == "sensor_msgs/CompressedImage") {
        const sensor_msgs::CompressedImageConstPtr img = 
          m.instantiate<sensor_msgs::CompressedImage>();
        if (syncImages_) {
          image_sync.process(m.getTopic(), img);
        } else
          processImage(m.getTopic(), img);
      } else if (m.getDataType() == "sensor_msgs/PointCloud2") {
        const sensor_msgs::PointCloud2ConstPtr pc = 
          m.instantiate<sensor_msgs::PointCloud2>();
        if (syncLidarWithImages_ && syncImages_)
          lastPointCloud_ = pc;
        else
          processPointCloud(m.getTopic(), pc);
      } else if (m.getDataType() == "sensor_msgs/Imu") {
        const sensor_msgs::ImuConstPtr imu = 
          m.instantiate<sensor_msgs::Imu>();
        processImu(imu);
      }
    }
  }

  void ExportBag::createDirectories() {
    //name folders
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
   
    n = 0; 
    for (auto &topic : pointCloudTopics_) {
      dir = "pointcloud"+std::to_string(n);
      topicNames_.insert(topicNames_.begin(), std::pair<std::string, std::string>(
            topic, dir));
      mkdir((outputDir_+"/"+dir).c_str(), 0777);
      mkdir((outputDir_+"/"+dir+"/data").c_str(), 0777);
      n++;
    }

    mkdir((outputDir_+"/imu").c_str(), 0777); 
  }

  void ExportBag::processSyncImages(const std::vector<sensor_msgs::CompressedImageConstPtr> &img_vec) {
    int i=0;
    for (const auto img : img_vec) {
      processImage(imageTopics_[i++], img);
    }

    if (lastPointCloud_ != NULL && syncImages_ && syncLidarWithImages_)
      processPointCloud(pointCloudTopics_[0], lastPointCloud_);
  }

  void ExportBag::processImage(const std::string &topic,
      const sensor_msgs::CompressedImageConstPtr &img) {
    cv::Mat origImage = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8)->image; 
    cv::Mat colorImage;
    //debayer
    cv::cvtColor(origImage, colorImage, cv::COLOR_BayerBG2BGR);

    if (useCLAHE_) {
      //convert to ycb, split up channels
      cv::Mat ycbImage;
      cv::cvtColor(colorImage, ycbImage, cv::COLOR_BGR2YCrCb);
      std::vector<cv::Mat> ycbChannels(3);
      cv::split(ycbImage, ycbChannels);
      //CLAHE on intensity channel
      clahe_->apply(ycbChannels[0], ycbChannels[0]);
      cv::merge(ycbChannels, ycbImage);
      cv::cvtColor(ycbImage, colorImage, cv::COLOR_YCrCb2BGR);
    }

    if (rectify_) {
      rectifyImage(colorImage, colorImage, topic);
    }

    int count = topicCounts_.at(topic);
    std::ostringstream stream;
    stream << outputDir_ << "/";
    stream << topicNames_.at(topic) << "/data/";
    stream << std::setfill('0') << std::setw(8) << count << ".jpg";
    
    //timestamp
    std::ofstream timestamp;
    timestamp.open(outputDir_+"/"+topicNames_.at(topic)+"/timestamps.txt", std::ios_base::app);
    timestamp << img->header.stamp << std::endl;

    ROS_INFO_STREAM(stream.str());

    cv::imwrite(stream.str(), colorImage);

    topicCounts_.at(topic) = count + 1;
  }

  void ExportBag::initRectifyMaps(const std::string &rectify_path) {
    YAML::Node trans_yaml = YAML::LoadFile(rectify_path);
    for (std::size_t i=0; i<imageTopics_.size(); i++) {
      std::string topic = "cam"+std::to_string(i);
      cv::Mat K = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
      K.at<double>(0, 0) = trans_yaml[topic]["intrinsics"][0].as<double>();   
      K.at<double>(1, 1) = trans_yaml[topic]["intrinsics"][1].as<double>();   
      K.at<double>(0, 2) = trans_yaml[topic]["intrinsics"][2].as<double>();   
      K.at<double>(1, 2) = trans_yaml[topic]["intrinsics"][3].as<double>();   
      K.at<double>(2, 2) = 1;

      cv::Mat distCoeffs = cv::Mat(4, 1, CV_64F, cv::Scalar(0));
      for (int j=0; j<4; j++)
        distCoeffs.at<double>(j, 0) = trans_yaml[topic]["distortion_coeffs"][j].as<double>(); 

      cv::Size size;
      size.width = trans_yaml[topic]["resolution"][0].as<double>();
      size.height = trans_yaml[topic]["resolution"][1].as<double>();

      cv::Mat map1, map2;
      cv::fisheye::initUndistortRectifyMap(K, distCoeffs, cv::Mat(), K, size,
          CV_16SC2, map1, map2);

      rectifyMap1_[imageTopics_[i]] = map1;
      rectifyMap2_[imageTopics_[i]] = map2;
    }
  }

  void ExportBag::rectifyImage(cv::Mat &in, cv::Mat &out, const std::string& cam) {
    cv::remap(in, out, rectifyMap1_[cam], rectifyMap2_[cam], cv::INTER_LINEAR);
  }

  void ExportBag::processPointCloud(const std::string &topic,
      const sensor_msgs::PointCloud2ConstPtr &pc) {
    //convert to pcl
    pcl::PointCloud<pcl::PointXYZI> pcl_pc;
    pcl::fromROSMsg<pcl::PointXYZI>(*pc, pcl_pc);

    int count = topicCounts_.at(topic);
    std::ostringstream stream;
    stream << outputDir_ << "/";
    stream << topicNames_.at(topic) << "/data/";
    stream << std::setfill('0') << std::setw(8) << count << ".pcd";
    //timestamp
    std::ofstream timestamp;
    timestamp.open(outputDir_+"/"+topicNames_.at(topic)+"/timestamps.txt", std::ios_base::app);
    timestamp << pc->header.stamp << std::endl;

    ROS_INFO_STREAM(stream.str());

    pcl::io::savePCDFileASCII(stream.str(), pcl_pc);

    topicCounts_.at(topic) = count + 1;
  }

  void ExportBag::processImu(const sensor_msgs::ImuConstPtr &imu) {
    std::ofstream imu_file;
    imu_file.open(outputDir_+"/imu/imu.txt", std::ios_base::app);
    imu_file << imu->orientation.x << "," << imu->orientation.y << 
      "," << imu->orientation.z << "," << imu->orientation.w << ",";
    imu_file << imu->angular_velocity.x << "," << imu->angular_velocity.y << 
      "," << imu->angular_velocity.z << ",";
    imu_file << imu->linear_acceleration.x << "," << imu->linear_acceleration.y <<
      "," << imu->linear_acceleration.z << ",";
    imu_file << imu->header.stamp << std::endl;
  }
}
