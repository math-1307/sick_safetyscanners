#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>

using namespace std;

class SickSubscriber    {
  private:
    ros::NodeHandle n;
    ros::Subscriber sub;

  public:
    SickSubscriber()   {
        
        sub = n.subscribe<sensor_msgs::LaserScan>("/laser_scan", 1, &SickSubscriber::sick_sensor_cb, this);
    }

    void arry2oss(ostringstream& oss, const vector<float>& arr) {
        oss << "[";
        for(size_t i = 0; i < arr.size(); i++)   {
            oss << arr[i];
            if(i == arr.size() - 1)    {
                oss << "]";
            } else  {
                oss << ", ";
            }
        }
    }

    void sick_sensor_cb(const sensor_msgs::LaserScan::ConstPtr& scan)   {
        if(!(scan->ranges.empty()))  {
            ostringstream ranges_oss, intensities_oss;
            this->arry2oss(ranges_oss, scan->ranges);
            this->arry2oss(intensities_oss, scan->intensities);

            ROS_INFO_STREAM("Laser Scan Data:"  << endl <<
                            "Header: "          << scan->header             <<
                            "Angle Min: "       << scan->angle_min          << endl <<
                            "Angle Max: "       << scan->angle_max          << endl <<
                            "Angle Increment: " << scan->angle_increment    << endl <<
                            "Time Increment: "  << scan->time_increment     << endl <<
                            "Scan Time: "       << scan->scan_time          << endl <<
                            "Range Min: "       << scan->range_min          << endl <<
                            "Range Max: "       << scan->range_max          << endl <<
                            "Ranges: "          << ranges_oss.str()         << endl <<
                            "Intensities: "     << intensities_oss.str()    << endl);
        } else {
            ROS_INFO_STREAM("Scan is empty!");
        }
    }
};

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "sick_subscriber");
    SickSubscriber sick_subscriber;
    ros::spin();
    return 0;
}