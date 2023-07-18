#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "sick_publisher");
    ros::NodeHandle nh;

    // Create a publisher for the LaserScan message
    ros::Publisher laserScanPub = nh.advertise<sensor_msgs::LaserScan>("/laser_scan", 3);

    // Set up the LaserScan message
    sensor_msgs::LaserScan laserScanMsg;
    laserScanMsg.header.frame_id = "laser_frame";
    laserScanMsg.angle_min = -1.57;   // Minimum angle in radians (-90 degrees)
    laserScanMsg.angle_max = 1.57;    // Maximum angle in radians (90 degrees)
    laserScanMsg.angle_increment = 0.3; // Angle increment in radians
    laserScanMsg.time_increment = 0.0;
    laserScanMsg.scan_time = 0.1;     // Time between scans in seconds
    laserScanMsg.range_min = 0.0;     // Minimum range value in meters
    laserScanMsg.range_max = 10.0;    // Maximum range value in meters

    // Set up the range values (example: constant range of 2.0 meters)
    size_t num_ranges = (laserScanMsg.angle_max - laserScanMsg.angle_min) / laserScanMsg.angle_increment + 1;
    laserScanMsg.ranges.resize(num_ranges, 2.0);
    laserScanMsg.intensities.resize(num_ranges, 1.0);

    // Publishing loop
    ros::Rate rate(10); // 10 Hz (publish every 0.1 seconds)
    while (ros::ok()) {
        laserScanMsg.header.stamp = ros::Time::now();
        laserScanPub.publish(laserScanMsg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
