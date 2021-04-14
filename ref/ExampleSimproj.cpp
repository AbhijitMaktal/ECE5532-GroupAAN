// ROS and node class header file
//This code demonstrates shortest path from intersection0 to intersection3
#include <ros/ros.h>
#include <ugv_course_libs/gps_conv.h>
#include <std_msgs/Float64.h>
#include <ugv_course_libs/gps_conv.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#define INF 0x3f3f3f3f
#define V 17
#define earthRadiusKm 6371.0
#define destLoc 3

// Set Destination Node here Manually not automated
struct road
{
double lat[3];
double lon[3];
tf::Vector3 local_navpoints[3];
Before intersections, Left, Right
}IntSecs[17];
UTMCoords ref_point;
ros::Publisher pub_wheel_angle;
//Before intersections, Left, Right
//Before intersections, Left, Right
//Coordinates converted into UTM;
//No of Total intersections, 17ros::Publisher pub_wheel_speed;
ros::Publisher pub_wheel_brake;
std_msgs::Float64 steering_angle_msg;
std_msgs::Float64 steering_speed_msg;
std_msgs::Float64 steering_brake_msg;
tf::Vector3 relative_position;
uint8_t cnt = 0;
uint32_t cntRow=0;
uint32_t cntCol=1;
int TestSeq[V];
int TestSeq2D[V][V];
int TestCmd[V] = //{0, 1, 1, 2, 3}; //for Intersection no 6
//{0, 1, 0, 1, 2, 0, 3} //for Intersection no 11
{0, 1, 1, 0, 2, 3};
//for Intersection no 2 //
Hardcoded sequence of commands
uint8_t Start = 0;
started
uint8_t NextIntSec = TestSeq[cnt+1];
Initialize to Starting intersection 1
uint8_t Command = 0;
Right = 2, STOP = 3
uint8_t ArrIntsec = 0;
int directedNode[V][V]={
{1,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0},
{1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,1,1,0,0,0,0,0,1,0,0,0,0,0,0},
{0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,1,1,1,0,1,0,0,0,0,0,0,0,0},
{0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0},
BETWEEN NODES.
{1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,1,0},
{0,0,0,0,0,1,0,1,1,1,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0},
{0,0,0,1,0,0,0,0,0,0,1,0,1,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0},
{1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1}
};
double gamma_ = 17.3;
wheel angle and tire angle
double L = 3.0;
rear wheels
double rightR = 1.5;
//Path following node is
//Next intersaction;
//Straight = 0, Left = 1,
//Intersaction Arrived
// 1 - REPRESENTS THE DIRECT CONNECTION
//ratio between steering
//distnace between front and
//Right radiusdouble leftR = 10;
double recvangle;
double recvspeed;
//Left radius
//Received audibot_cmd Angle
//Received audibot_cmd Speed
int counter1=1;
int counter2=0;
//Dijkstra algorithm
int minDistance(int dist[], bool sptSet[])
{
// Initialize min value
int min = INT_MAX, min_index;
for (int v = 0; v < V; v++)
if (sptSet[v] == false && dist[v] <= min)
min = dist[v], min_index = v;
return min_index;
}
void printPath(int parent[], int j)
{
// Base Case : If j is source
if (parent[j]==-1)
return;
//
printPath(parent, parent[j]);
ROS_INFO("%d ", j);
TestSeq2D[cntRow][cntCol]=j;
cntCol++;
}
int printSolution(int dist[], int n, int parent[])
{
int src = 0;
//ROS_INFO("Vertex\t Distance\tPath");
for (int i = 1; i < V; i++)
{
//
ROS_INFO("\n%d -> %d \t\t %d\t\t%d ", src, i, dist[i], src);
printPath(parent, i);
cntRow++;
cntCol=1;
}
}
// Funtion that implements Dijkstra's single source shortest path
// algorithm for a graph represented using adjacency matrix
// representation
void dijkstra(int graph[V][V], int src)
{
int dist[V]; // The output array. dist[i] will hold
// the shortest distance from src to i
// sptSet[i] will true if vertex i is included / in shortest
// path tree or shortest distance from src to i is finalized
bool sptSet[V];// Parent array to store shortest path tree
int parent[V];
// Initialize all distances as INFINITE and stpSet[] as false
for (int i = 0; i < V; i++)
{
parent[0] = -1;
dist[i] = INT_MAX;
sptSet[i] = false;
}
// Distance of source vertex from itself is always 0
dist[src] = 0;
// Find shortest path for all vertices
for (int count = 0; count < V-1; count++)
{
// Pick the minimum distance vertex from the set of
// vertices not yet processed. u is always equal to src
// in first iteration.
int u = minDistance(dist, sptSet);
// Mark the picked vertex as processed
sptSet[u] = true;
// Update dist value of the adjacent vertices of the
// picked vertex.
for (int v = 0; v < V; v++)
//
//
//
//
if
Update dist[v] only if is not in sptSet, there is
an edge from u to v, and total weight of path from
src to v through u is smaller than current value of
dist[v]
(!sptSet[v] && graph[u][v] &&
dist[u] + graph[u][v] < dist[v])
{
parent[v] = u;
dist[v] = dist[u] + graph[u][v];
}
}
// print the constructed distance array
printSolution(dist, V, parent);
}
// This function converts decimal degrees to radians
double deg2rad(double deg) {
return (deg * 3.14159265358979323846 / 180);
};
// This function converts radians to decimal degrees
double rad2deg(double rad) {
return (rad * 180 / 3.14159265358979323846);
};// This function returns distance between provided source Lat, Lon and
Destination Lat, Lon
double distanceEarth(double latSrc, double lonSrc, double latDest, double
lonDest)
{
double latSrcRad, lonSrcRad, latDestRad, lonDestRad, u, v;
latSrcRad = deg2rad(latSrc);
lonSrcRad = deg2rad(lonSrc);
latDestRad = deg2rad(latDest);
lonDestRad = deg2rad(lonDest);
u = sin((latDestRad - latSrcRad)/2);
v = sin((lonDestRad - lonSrcRad)/2);
return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(latSrcRad) *
cos(latDestRad) * v * v));
}
// This Function is used to create a 2D array of distances between each point
void setGraphValues(road *IntSecs_array)
{
// Define Graph Setting variable
int graph[V][V];
double dist1 = 0.0;
for (int l=0; l<17; l++)
{
for (int m=0; m<17; m++)
{
// if( (l==1 and m==8) or (l==8 and m==9) or (l==9 and m==6) or (l==6
and m==5) or (l==5 and m==4) or (l==4 and m==3) or (l==9 and m==10) or (l==10
and m==11) or (l==11 and m==4) )
//{
if (directedNode[l][m]==1)
{
dist1 = distanceEarth(IntSecs_array[l].lat[0],
IntSecs_array[l].lon[0], IntSecs_array[m].lat[0], IntSecs_array[m].lon[0]);
int dist2 = (int) (dist1 * 100000);
graph[l][m] = dist2;
}
else
{
graph[l][m] = INF;
}
}
}
dijkstra(graph, 0);
int SelectedRow=-1;
//
ROS_INFO("\nStored
Array:\n\n============================================================\n\n");
//
for (int i=0; i<V; i++)
//
{
//
for (int j=0; j<V; j++)
//
{
// //
ROS_INFO(" %d ",TestSeq2D[i][j]);//
// //
//
}
ROS_INFO("\n");
}
// Select Row Number K, store it in SelectedRow --
for (int k=0; k<V; k++)
{
for (int l=0; l<V; l++)
{
//
ROS_INFO("%d\t%d\t%d\t", TestSeq2D[k][l], destLoc,
TestSeq2D[k][l+1]);
if(TestSeq2D[k][l] == destLoc && TestSeq2D[k][l+1]==-1)
{
SelectedRow=k;
}
}
}
// Based on Selected Row Number K, Select All Elements of K Row into
TestSeq 1D array
for (int l=0; l<V; l++)
{
ROS_INFO(" (%d) ", TestSeq2D[SelectedRow][l]);
//while (TestSeq2D[SelectedRow][l] != -1)
TestSeq[l] = TestSeq2D[SelectedRow][l];
}
NextIntSec = TestSeq[cnt+1];
// Added to get next intersection number
// Final Selected Array
ROS_INFO("Final TestSeq : \n");
}
//Receive current position of vehicle
void recvFix(const sensor_msgs::NavSatFixConstPtr& msg)
{
// Convert current geodetic coordinates into UTM
UTMCoords current_utm(*msg);
// Use overloaded - operator to subtract two UTM objects to
// get the relative position as a tf::Vector3
relative_position = current_utm - ref_point;
}
void recvSteercmd(const geometry_msgs::TwistConstPtr& msg)
{
recvspeed = msg->linear.x;
//receive speed from path following
recvangle = msg->angular.z;
//receive yaw rate from path following
steering_angle_msg.data = gamma_*atan((L*recvangle)/recvspeed);
steering_speed_msg.data = 0.1;
steering_brake_msg.data = 0;
if(Command >= 3)
//set speed
//set brake{
steering_speed_msg.data = 0;
//speed when audibot should sTOP
steering_brake_msg.data = 20000;
//brake when audibot should sTOP
}
if(Command == 0)
{
pub_wheel_angle.publish(steering_angle_msg);
}
pub_wheel_speed.publish(steering_speed_msg);
pub_wheel_brake.publish(steering_brake_msg);
Start = 1;
}
void timerCallback(const ros::TimerEvent& event)
{
if(Start == 1) // Start if path following node is started
{
switch(Command)
//Turn direction
{
case 0: //Straight
ROS_INFO("Straight -> \tNextIntSec=%d",NextIntSec);
ROS_INFO("Y=%f \t X=%f",fabs(relative_position.getY() -
IntSecs[NextIntSec].local_navpoints[Command].y()),
fabs(relative_position.getX() -
IntSecs[NextIntSec].local_navpoints[Command].x()) );
steering_angle_msg.data = gamma_*atan((L*recvangle)/recvspeed);
break;
case 1: //Left
ROS_INFO("Left ->\tY=%f \t X=%f",fabs(relative_position.getY() -
IntSecs[NextIntSec].local_navpoints[Command].y()),
fabs(relative_position.getX() -
IntSecs[NextIntSec].local_navpoints[Command].x()) );
steering_angle_msg.data = gamma_*atan(L / leftR);
break;
case 2: //Right
ROS_INFO("Right ->\tY=%f \t X=%f",fabs(relative_position.getY() -
IntSecs[NextIntSec].local_navpoints[Command].y()),
fabs(relative_position.getX() -
IntSecs[NextIntSec].local_navpoints[Command].x()) );
steering_angle_msg.data = -gamma_*atan(L / rightR);
//-ve for right
turn
steering_brake_msg.data = 7000;
break;
default: //STOP
ROS_INFO("Case Default -> STOP");
steering_angle_msg.data = 0;
steering_speed_msg.data = 0;
steering_brake_msg.data = 15000;
break;}
if( ( fabs(relative_position.getY() -
IntSecs[NextIntSec].local_navpoints[Command].y()) <= 2.5) && (
fabs(relative_position.getX() -
IntSecs[NextIntSec].local_navpoints[Command].x()) <= 2.5) && (Command < 3) )
{
if(ArrIntsec == 0)
{
Command = TestCmd[cnt+1];
//increment command count
ArrIntsec = !(ArrIntsec);
//Intersection reached
ROS_INFO("ArrIntsec = %d\tCommand =
%d\tNextIntSec=%d",ArrIntsec,Command,NextIntSec);
}
else if(ArrIntsec == 1)
{
++cnt;
//increment to get next
intersections
Command = 0;
//Go Straight
NextIntSec = TestSeq[cnt+1];
//increment intersection
count
ArrIntsec = !(ArrIntsec);
//Turn Complete
ROS_INFO("Cnt=%d\tArrIntsec = %d\tCommand =
%d\tNextIntSec=%d",cnt,ArrIntsec,Command,NextIntSec);
}
}
else if (Command >= 3)
{
steering_angle_msg.data = 0;
//
pub_wheel_angle.publish(steering_angle_msg);
steering_speed_msg.data = 0;
steering_brake_msg.data = 20000;
}
}// End of if(Start == 1)
pub_wheel_angle.publish(steering_angle_msg);
pub_wheel_speed.publish(steering_speed_msg);
pub_wheel_brake.publish(steering_brake_msg);
}// End of timer
void setCoordinates()
{
memset(&IntSecs, 0, sizeof(road));
IntSecs[0].lat[0] = 42.8558957098;
IntSecs[1].lat[0] = 42.8560945832;
IntSecs[2].lat[0] = 42.8561367777;
IntSecs[3].lat[0] = 42.8547097543;
IntSecs[3].lat[1] = 42.8548332007;
IntSecs[3].lat[2] = 42.8549318552;
IntSecs[4].lat[0] = 42.8538005257;IntSecs[4].lat[2] = 42.8538466287;
IntSecs[5].lat[0] = 42.8538153195;
IntSecs[5].lat[2] = 42.8538881055;
IntSecs[6].lat[0] = 42.8543104839;
IntSecs[7].lat[0] = 42.8532145518;
IntSecs[7].lat[1] = 42.8533037854;
IntSecs[8].lat[0] = 42.8537534434;
IntSecs[8].lat[1] = 42.8538354064;
IntSecs[9].lat[0] = 42.8547870694;
IntSecs[9].lat[1] = 42.8548680012;
IntSecs[10].lat[0] = 42.8548584015;
IntSecs[10].lat[2] = 42.8549062736;
IntSecs[11].lat[0] = 42.8562493118;
IntSecs[12].lat[0] = 42.8566605387;
IntSecs[12].lat[2] = 42.8566172534;
IntSecs[13].lat[0] = 42.8561982968;
IntSecs[14].lat[0] = 42.8546931533;
IntSecs[15].lat[0] = 42.8559252172;
IntSecs[15].lat[1] = 42.8560398807;
IntSecs[15].lat[2] = 42.856004024;
IntSecs[16].lat[0] = 42.8551857938;
//longitudes
IntSecs[0].lon[0] = -83.07240914;
IntSecs[1].lon[0] = -83.0704434705;
IntSecs[2].lon[0] = -83.0702692682;
IntSecs[3].lon[0] = -83.0677087319;
IntSecs[3].lon[1] = -83.0678600121;
IntSecs[4].lon[0] = -83.0675725944;
IntSecs[4].lon[2] = -83.067682663;
IntSecs[5].lon[0] = -83.0668134283;
IntSecs[5].lon[2] = -83.0668863526;
IntSecs[6].lon[0] = -83.0668933705;
IntSecs[7].lon[0] = -83.0655712598;
IntSecs[7].lon[1] = -83.0654632145;IntSecs[8].lon[0] = -83.0654834083;
IntSecs[8].lon[1] = -83.0655985502;
IntSecs[9].lon[0] = -83.0655121546;
IntSecs[9].lon[1] = -83.0656320239;
IntSecs[10].lon[0] = -83.0662438802;
IntSecs[10].lon[2] = -83.0663054467;
IntSecs[11].lon[0] = -83.0668314714;
IntSecs[12].lon[0] = -83.0676167528;
IntSecs[12].lon[2] = -83.0675532168;
IntSecs[13].lon[0] = -83.0675305768;
IntSecs[14].lon[0] = -83.0649460973;
IntSecs[15].lon[0] = -83.0642305489;
IntSecs[15].lon[1] = -83.0643920262;
IntSecs[15].lon[2] = -83.064134928;
IntSecs[16].lon[0] = -83.0629480864;
IntSecs[5].lon[0] = -83.0667750545;
// IntSecs[5].lon[1] = -83.0668858016;
for (int i = 0; i < 17; i++)
{
for (int j = 0; j < 3; j++)
{
UTMCoords waypoint(LatLon(IntSecs[i].lat[j], IntSecs[i].lon[j], 0.0));
IntSecs[i].local_navpoints[j] = waypoint - ref_point;
//
ROS_INFO("LocalPoints: %f",IntSecs[i].local_navpoints[j]);
}
}
for (int i=0; i<V; i++)
for (int j=0; j<V; j++)
TestSeq2D[i][j]=-1;
for (int i=0; i<V; i++)
TestSeq2D[i][0]=1;
ROS_INFO("Calling SetGraphValues Function...\n");
setGraphValues(IntSecs);
}
int main(int argc, char** argv)
{
// Initialize ROS and declare node handles
ros::init(argc, argv, "audibot_road_world");ros::NodeHandle n;
ros::NodeHandle pn("~");
//Subscibers
ros::Subscriber sub_fix = n.subscribe("/audibot/gps/fix", 1, recvFix);
ros::Timer marker_timer = n.createTimer(ros::Duration(0.05),
timerCallback);
ros::Subscriber sub_steer = n.subscribe("/roadsteer_cmd", 1, recvSteercmd);
//Publishers
pub_wheel_angle = n.advertise<std_msgs::Float64>("/audibot/steering_cmd",
1);
pub_wheel_speed = n.advertise<std_msgs::Float64>("/audibot/throttle_cmd",
1);
pub_wheel_brake = n.advertise<std_msgs::Float64>("/audibot/brake_cmd", 1);
double ref_lat, ref_lon;
n.param("/audibot/gps/ref_lat", ref_lat, 0.0);
n.param("/audibot/gps/ref_lon", ref_lon, 0.0);
// Initialize ROS and declare node handles
ros::init(argc, argv, "audibot_road_world");ros::NodeHandle n;
ros::NodeHandle pn("~");
//Subscibers
ros::Subscriber sub_fix = n.subscribe("/audibot/gps/fix", 1, recvFix);
ros::Timer marker_timer = n.createTimer(ros::Duration(0.05),
timerCallback);
ros::Subscriber sub_steer = n.subscribe("/roadsteer_cmd", 1, recvSteercmd);
//Publishers
pub_wheel_angle = n.advertise<std_msgs::Float64>("/audibot/steering_cmd",
1);
pub_wheel_speed = n.advertise<std_msgs::Float64>("/audibot/throttle_cmd",
1);
pub_wheel_brake = n.advertise<std_msgs::Float64>("/audibot/brake_cmd", 1);
double ref_lat, ref_lon;
n.param("/audibot/gps/ref_lat", ref_lat, 0.0);
n.param("/audibot/gps/ref_lon", ref_lon, 0.0);
ref_point = UTMCoords(LatLon(ref_lat, ref_lon, 0.0));
setCoordinates();
// Spin and process callbacks
ros::spin();
return 0;
}
ref_point = UTMCoords(LatLon(ref_lat, ref_lon, 0.0));
setCoordinates();
// Spin and process callbacks
ros::spin();
return 0;
}