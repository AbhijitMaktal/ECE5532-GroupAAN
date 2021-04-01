#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <ugv_course_libs/gps_conv.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


UTMCoords ref_coords;
tf::Vector3 relative_position;
nav_msgs::Path gps_path;
ros::Publisher path_pub;

float veh_heading;
ros::Publisher pub_vel;
ros::Publisher pub_steering;

UTMCoords waypoint_coords;

std::vector<double> waypoint_lat;
std::vector<double> waypoint_lon;
std::vector<tf::Vector3> relative_position_waypoint;
int current_waypoint;


ros::Publisher pub_markers;
ros::Publisher pub_angles;
ros::Publisher pub_angles2;
ros::Publisher pub_angles3;

double veh_lat;
double veh_lon;
double central_meridian;

geometry_msgs::Twist cmd_vel; //should this be in the structure? 
std_msgs::Float64 cmd_steering;

visualization_msgs::MarkerArray marker_array_msg; 
visualization_msgs::Marker marker;

void timerCallback(const ros::TimerEvent& event){
   //should only init

  waypoint_lat.resize(8);
  waypoint_lat[0] = 42.851358;
  waypoint_lat[1] = 42.851383;
  waypoint_lat[2] = 42.852443;
  waypoint_lat[3] = 42.852021;
  waypoint_lat[4] = 42.851525;
  waypoint_lat[5] = 42.851344;
  waypoint_lat[6] = 42.850836;
  waypoint_lat[7] = 42.849644;
  waypoint_lon.resize(8);
  waypoint_lon[0] = -83.069485;
  waypoint_lon[1] = -83.069007;
  waypoint_lon[2] = -83.068013;
  waypoint_lon[3] = -83.066888;
  waypoint_lon[4] = -83.067044;
  waypoint_lon[5] = -83.066344;
  waypoint_lon[6] = -83.066440;
  waypoint_lon[7] = -83.066060;

  relative_position_waypoint.resize(8);
  for (int k = 0; k < 8; k++) {
    UTMCoords waypoint(LatLon(waypoint_lat[k], waypoint_lon[k], 0.0));
    relative_position_waypoint[k] = waypoint - ref_coords;
  }

  geometry_msgs::PoseStamped current_pose;
  current_pose.pose.position.x = relative_position.x();
  current_pose.pose.position.y = relative_position.y();
  //Use i to shorten code
  int i = current_waypoint; 

  gps_path.poses.push_back(current_pose);
  gps_path.header.frame_id="world";
  gps_path.header.stamp = event.current_real;
  path_pub.publish(gps_path);
  
  double dist = (relative_position - relative_position_waypoint[i]).length();
  double theta = atan2((relative_position_waypoint[i].y() - relative_position.y()),(relative_position_waypoint[i].x() - relative_position.x()));

  double veh_lon_rad = ((veh_lon*M_PI)/180); 
  double veh_lat_rad = ((veh_lat*M_PI)/180); 
  double central_meridian_rad = ((central_meridian*M_PI)/180); 

  double convergence_angle = atan(tan(veh_lon_rad-central_meridian_rad)*sin(veh_lat_rad));
  double veh_angle_rad = ((veh_heading*M_PI)/180);//Convert to radian
  double veh_angle_raw = (M_PI_2 - veh_angle_rad);//Covert to ROS coordiantes from true north
  double veh_angle = veh_angle_raw + convergence_angle;

  double speed;
  double steering_gain;
  double error_steering;
  error_steering = theta-veh_angle;

  //account for large angles, only works for up to 3pi/2
  if (error_steering > M_PI){
    error_steering = error_steering - (2*M_PI);
  }else if (error_steering < (-M_PI)){
    error_steering = error_steering +(2*M_PI);
  }

  //Nick removed Control model since it was not great. 
  //
  
  cmd_vel.linear.x =  speed;
  cmd_steering.data = steering_gain;//*(error_steering);

  pub_vel.publish(cmd_vel);
  pub_steering.publish(cmd_steering);

  //Use to use rqt to troubleshoot angle calculation
  std_msgs::Float64 cmd_theta;
  cmd_theta.data = theta;
  pub_angles.publish(cmd_theta);

  std_msgs::Float64 cmd_error;
  cmd_error.data = error_steering;
  pub_angles.publish(cmd_error);

   std_msgs::Float64 cmd_vehicle;
  cmd_vehicle.data = veh_angle;
  pub_angles.publish(cmd_vehicle);

/*
  if (dist<0.99){ //Set threshold below 1 
    current_waypoint++; //Is this correct???
  }
*/
/*
  ROS_INFO("Relative Position: (%f, %f)", relative_position.x(), relative_position.y());
  //ROS_INFO("Waypoint UTM: (%f, %f)", waypoint_coords.getX(), waypoint_coords.getY());
  ROS_INFO("Waypoint Relative Position: (%f, %f)", relative_position_waypoint[i].x(), relative_position_waypoint[i].y());

  ROS_INFO("Calculated Distance: (%f)",  dist);
  //ROS_INFO("Angles: (%f, %f)", theta, veh_heading);
  ROS_INFO("Steering: (%f)", cmd_steering.data);
  ROS_INFO("Error Ster: (%f)", error_steering);
  ROS_INFO("Theta : (%f)", theta);
  ROS_INFO("veh ang : (%f)", veh_angle);
  //ROS_INFO("veh heading : (%f)", veh_heading);
  //ROS_INFO("veh rad : (%f)", veh_angle_rad);
  //ROS_INFO("veh raw : (%f)", veh_angle_raw);
  
  ROS_INFO("convergence_angle : (%f)", convergence_angle);
  ROS_INFO("Current waypoint: (%d)",current_waypoint);
  //ROS_INFO("Current waypoint: (%d)",current_waypoint);
*/
}

void recvFix(const sensor_msgs::NavSatFixConstPtr& msg){
  UTMCoords current_coords(*msg);
  veh_lat = msg->latitude;
  veh_lon = msg->longitude;
  relative_position = current_coords - ref_coords;
//Removed Marker Array
  
}

void recvFix2(const std_msgs::Float64ConstPtr& msg){
  veh_heading = msg->data;
}


int main(int argc, char** argv){
  ros::init(argc,argv,"gps_sim_nhabben");
  ros::NodeHandle nh;
  ros::Subscriber gps_heading = nh.subscribe("/audibot/gps/fix",1,recvFix);
  ros::Subscriber gps_sub = nh.subscribe("/audibot/gps/heading",1,recvFix2);
  ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCallback);
  path_pub = nh.advertise<nav_msgs::Path>("gps_path",1);

  double ref_lat;
  double ref_lon;
  current_waypoint = 0;

  nh.getParam("/audibot/gps/ref_lat",ref_lat);
  nh.getParam("/audibot/gps/ref_lon",ref_lon);
  
  LatLon ref_coords_lat_lon(ref_lat, ref_lon, 0);
  ref_coords = UTMCoords(ref_coords_lat_lon);

  central_meridian = ref_coords.getCentralMeridian();

  ROS_INFO("Central Meridian of the Reference Cooridinate: %f", central_meridian);
  ROS_INFO("ref lat and ref lon %f  %f",ref_lat,ref_lon); //Add in for checking init location

  pub_vel = nh.advertise<geometry_msgs::Twist>("/audibot/cmd_vel", 1);
  pub_steering = nh.advertise<std_msgs::Float64>("/audibot/steering_cmd", 1);

  ros::spin();
}
