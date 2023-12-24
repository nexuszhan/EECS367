#include "Fetch_Controller.hpp"

Fetch_Controller::Fetch_Controller(ros::NodeHandle &nh)
{
    nh_ = nh;

    //TODO: initialize a subscriber that is set to the channel "/base_scan". Set its callback function to be Laser_Scan_Callback
    //TODO: initialize a publisher that is set to the channel "/cmd_vel"
    subscriber_ = nh_.subscribe("/base_scan", 1, &Fetch_Controller::Laser_Scan_Callback, this);
    publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void Fetch_Controller::Laser_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr &msg_laser_scan)
{
    /*TODO: 
    Given the incoming laser scan message, find the minimium distance of the front facing scans
    Hint: The laser scan measuring directly in front of the robot will be the scan at the middle of the array laser scans. 
    So for finding the minimum, we will ONLY consider the 120 laser scans in the middle of the array of laser scans. 
    If the minimum scan in this direction is greater than 1m, drive forward. 
    Otherwise, turn left. 
    */
   int length = msg_laser_scan->ranges.size();
   int start = (length - 120) / 2;
   float min_dist = msg_laser_scan->ranges[start];
   for (int i=0; i<120; i++)
   {
        min_dist = std::min(min_dist, msg_laser_scan->ranges[start+i]);
   }

   geometry_msgs::Twist msg; 
   if (min_dist > 1.0)
   {
       msg.linear.x = 0.5;
       msg.linear.y = msg.linear.z = 0;
       msg.angular.x = msg.angular.y = msg.angular.z = 0;
   }
   else
   {
       msg.linear.x = msg.linear.y = msg.linear.z = 0;
       msg.angular.x = msg.angular.y = 0;
       msg.angular.z = 1.0;
   }
   publisher_.publish(msg);
}
