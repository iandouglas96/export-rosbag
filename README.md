# export-rosbag

Utility to export a rosbag into a directory structure similar to KITTI.

## Dependencies

I have only tested with ROS Melodic and Ubuntu 18.04, though it will likely work with earlier versions as well.  Aside from the ROS dependencies, OpenCV and PCL are both also required, to handle images and pointclouds, respectively.

## Usage

export-rosbag is built as a ROS node.  If all dependencies are in order, just create a catkin workspace, add export-rosbag, and run `catkin_make`.

All configuration is done in `launch/export_bag.launch`.  Provide the path to the relevant bag file, as well as the various topics you would like to export.  Currently images, pointclouds, and imu data are supported.  Multiple images and pointclouds can be exported simultaneously.  Additionally, make sure that you provide a path to an empty folder to export to.

Finally, run `roslaunch export_bag export_bag.launch` to run the export.  It may take a while.

## Output file structure

The output file structure will be created automatically.  Each topic will be mapped to a folder (i.e. image2, pointcloud0).  Each folder has a data subfolder with the sequential exported data (.jpg for images, .pcd (PCL Pointcloud) for pointclouds).  A `timestamps.txt` file gives the time each image was logged in seconds since epoch.

IMU data is logged somewhat differently to `imu.txt`.  This is essentially a csv file.  The first 4 columns are orientation (quaternion xyzw), the next 3 angular velocity (about xyz in rad/sec), and the next 3 linear acceleration (xyz m/s^2).  The final column in timestamp, in seconds since epoch.

## TODO

- Option to rectify images
- Option to filter images (CLAHE)
- Option to synchronize images and pointclouds
- Export pointclouds to KITTI format
