#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


bool itemPicked = false;
bool itemDropped = false;

double pick_X = 3.0;
double pick_Y = 3.0;
double drop_X = -3.0;
double drop_Y = 3.0;

// Create a function call to odometry data to get the robot current X & Y coordinate data
void odomData(const nav_msgs::Odometry::ConstPtr& msg) {
  double robotX = msg->pose.pose.position.x;
  double robotY = msg->pose.pose.position.y;
  
  // Caluclate the distance to Pickup location and drop off location based on the mathematical formula for distance of a line
  double pickup_zone = abs(robotX - pick_X) * abs(robotY - pick_Y);  // Estimate of distance in X, Y coordinates from Pick up point
  double dropoff_zone = abs(robotX - drop_X) * abs(robotY - drop_Y); // Estimate of distance in X, Y coordinates from Pick up point

  if (pickup_zone < 0.1) {
    itemPicked = true; // Distance smaller than a value means the robot has reached the defined pick up location
  }
  
  if (dropoff_zone < 0.1) {
    itemDropped = true; // Distance smaller than a value means the robot has reached the defined drop off location
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 5, odomData);

  visualization_msgs::Marker marker;

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "cube";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pick_X;
  marker.pose.position.y = pick_Y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  while (ros::ok())
  {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    if (!itemPicked)
    {
      // Add marker if Item is not picked up
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
    }
    else if (!itemDropped)
    {
      // Delete marker if item is dropped off
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
    }
    else
    {
      // Defaults setting
      marker.pose.position.x = drop_X;
      marker.pose.position.y = drop_Y;
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
    }
    
    ros::spinOnce();
  }
}
