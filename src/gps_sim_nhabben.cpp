#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <ugv_course_libs/gps_conv.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <math.h>
//#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <bits/stdc++.h>

using namespace std;

#define ROW 24
#define COL 15

//-------------------------------------------
//-------------------------------------------
//INputs
    // Add A*
  //Set the target Intersection
  string TargetIntersection = "D";
  //Set the Starting Car Location 
  string StartingIntersection = "A";
  //Default Location is A
//-------------------------------------------
//-------------------------------------------

UTMCoords ref_coords;
tf::Vector3 relative_position;

float veh_heading;
ros::Publisher pub_vel;
ros::Publisher pub_steering;
ros::Publisher pub_brake;

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

//Comment to check if can delete
// visualization_msgs::MarkerArray marker_array_msg; 
// visualization_msgs::Marker marker;

//Urban Nav Variables
double pathSpeed; double pathAngle;double pathSteeringAngle;
//Gam and L Used in Steering Calc
double gam = 17.3;
double L = 3;
std_msgs::Float64 cmd_throttle;
std_msgs::Float64 cmd_brake;

double atPoint = 0;

//list of points to travel
std::vector<double> waypoint_path;

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;
// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int> > pPair;
struct cell {
	// Row and Column index of its parent
	// Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	int parent_i, parent_j;
	// f = g + h
	double f, g, h;
};
//-------------

std::vector<double> waypointPath;
std::vector<int> waypointTurn;
int lastcellRow1; int lastcellCol1; int lastcellRow2; int lastcellCol2; int lastInter = 0; int PathTestCnt = -1;

int currentLoop = 0;

int grid[ROW][COL]
      = {
  {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, 
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 
  {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1}, 
  {1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1}, 
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, 
  {1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1}, 
  {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1}, 
  {1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1}, 
  {1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1}, 
  {1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1}, 
  {1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1}, 
  {1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1}, 
  {1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1}, 
  {1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1}, 
  {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1}, 
  {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1}, 
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, 
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, 
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, 
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 
  {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, 
  {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, 
  {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
};

int locationArray[17][2]={
	{20,7}, //A
	{6,14}, //B
	{6,12}, //C
	{10,12},//D
	{13,12},//E
	{13,8}, //F
	{16,6}, //G
	{8,8},  //H
	{6,8},  //I
	{12,4}, //J
	{10,6}, //K
	{12,6}, //L
	{10,10},//M
	{2,7},  //N
	{0,8},  //O
	{4,10}, //P
	{23,7}, //Q
};

int startx;int starty;int endx;int endy;

//A function to test if a given row, col exist as an intersection
void OnPathTest(int row,int col){
  
	PathTestCnt++;

	//Check for turn before deciding if the current point is an intersection
	if (lastInter){
		//Calculate Angle of Travel
		int delrow; int delcol;
		delrow = row-lastcellRow2;
		delcol = col-lastcellCol2;

		int turnDir;
		//Last point was an intersection, check for turn
		if(row == lastcellRow2 || col == lastcellCol2){
			//Straight
			turnDir = 0;
		}
		else if(lastcellRow1 == lastcellRow2) // East/West
		{
			double angleInter = atan(delrow/delcol);
			if(angleInter<0){
				//Left Turn}
				turnDir = -1;
			}
			else{
				//Right Turn
				turnDir = 1;
			}
		}
		else if(lastcellCol1 == lastcellCol2) // North/South
		{
			double angleInter = atan(delrow/delcol);
			if(angleInter>0){ //Positive is a Left Turn
				//Left Turn}
				turnDir = -1;
			}
			else{
				//Right Turn
				turnDir = 1;
			}
		}
		waypointTurn.push_back(turnDir);
		//Reset Trigger
		lastInter = 0;
	}
 
	if (PathTestCnt > 0){
		// for(int z = 0; z = sizeof(locationArray); z++){
		for(int z = 0; z < 17; z++){
			if (row == locationArray[z][0] && col == locationArray[z][1]){
				waypointPath.push_back(z);
				//printf("\nIntersection %d added \n",z);
        ROS_INFO("Astar search ran and found an intersection : (%d)",  z);
				lastInter = 1; //sets to true
			}
		}
	}
	
	if (!lastcellRow2)
	{
		lastcellRow2 = row;
		lastcellCol2 = col;
	}
	else{
	lastcellRow2 = lastcellRow1; 
	lastcellCol2 = lastcellCol1;
	}
	lastcellRow1 = row;
	lastcellCol1 = col;

	return;
}

// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool isValid(int row, int col)
{
	// Returns true if row number and column number
	// is in range
	return (row >= 0) && (row < ROW) && (col >= 0)
		&& (col < COL);
}

// A Utility Function to check whether the given cell is
// blocked or not
bool isUnBlocked(int grid[][COL], int row, int col)
{
	// Returns true if the cell is not blocked else false
	if (grid[row][col] == 1)
		return (true);
	else
		return (false);
}

// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(int row, int col, Pair dest)
{
	if (row == dest.first && col == dest.second)
		return (true);
	else
		return (false);
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, Pair dest)
{
	// Return using the distance formula
	return ((double)sqrt(
		(row - dest.first) * (row - dest.first)
		+ (col - dest.second) * (col - dest.second)));
}


// A Utility Function to trace the path from the source
// to destination
void tracePath(cell cellDetails[][COL], Pair dest)
{
	printf("\nThe Path is ");
	int row = dest.first;
	int col = dest.second;

	stack<Pair> Path;

	while (!(cellDetails[row][col].parent_i == row
			&& cellDetails[row][col].parent_j == col)) {
		Path.push(make_pair(row, col));
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
		row = temp_row;
		col = temp_col;
	}

	Path.push(make_pair(row, col));
	while (!Path.empty()) {
		pair<int, int> p = Path.top();
		Path.pop();
		printf("-> (%d,%d) ", p.first, p.second);
		//printf("Before OnPathTest");
		//Added to check if row, col are a intersection, add to global list
    ROS_INFO("\nwriting to path : ()");
		OnPathTest(p.first,p.second);
	}

	//printf("\n %d \n",Path.top());

	return;
}

// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
void aStarSearch(int grid[][COL], Pair src, Pair dest)
{

	// Create a closed list and initialise it to false which
	bool closedList[ROW][COL];
	memset(closedList, false, sizeof(closedList));

	// Declare a 2D array of structure to hold the details
	// of that cell
	cell cellDetails[ROW][COL];

	int i, j;

	for (i = 0; i < ROW; i++) {
		for (j = 0; j < COL; j++) {
			cellDetails[i][j].f = FLT_MAX;
			cellDetails[i][j].g = FLT_MAX;
			cellDetails[i][j].h = FLT_MAX;
			cellDetails[i][j].parent_i = -1;
			cellDetails[i][j].parent_j = -1;
		}
	}

	// Initialising the parameters of the starting node
	i = src.first, j = src.second;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
	cellDetails[i][j].parent_i = i;
	cellDetails[i][j].parent_j = j;
	/*
	Create an open list having information as-
	<f, <i, j>>
	where f = g + h,
	and i, j are the row and column index of that cell
  */
	set<pPair> openList;

	// Put the starting cell on the open list and set its
	// 'f' as 0
	openList.insert(make_pair(0.0, make_pair(i, j)));

	bool foundDest = false;

	while (!openList.empty()) {
		pPair p = *openList.begin();

		// Remove this vertex from the open list
		openList.erase(openList.begin());

		// Add this vertex to the closed list
		i = p.second.first;
		j = p.second.second;
		closedList[i][j] = true;


		//
		double gNew, hNew, fNew;

		//North
		if (isValid(i - 1, j) == true) {

			if (isDestination(i - 1, j, dest) == true) {
				cellDetails[i - 1][j].parent_i = i;
				cellDetails[i - 1][j].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}

			else if (closedList[i - 1][j] == false
					&& isUnBlocked(grid, i - 1, j)
							== true) {
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i - 1, j, dest);
				fNew = gNew + hNew;

				if (cellDetails[i - 1][j].f == FLT_MAX
					|| cellDetails[i - 1][j].f > fNew) {
					openList.insert(make_pair(
						fNew, make_pair(i - 1, j)));

					cellDetails[i - 1][j].f = fNew;
					cellDetails[i - 1][j].g = gNew;
					cellDetails[i - 1][j].h = hNew;
					cellDetails[i - 1][j].parent_i = i;
					cellDetails[i - 1][j].parent_j = j;
				}
			}
		}

		//South
		if (isValid(i + 1, j) == true) {
			if (isDestination(i + 1, j, dest) == true) {
				cellDetails[i + 1][j].parent_i = i;
				cellDetails[i + 1][j].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}
			else if (closedList[i + 1][j] == false
					&& isUnBlocked(grid, i + 1, j)
							== true) {
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i + 1, j, dest);
				fNew = gNew + hNew;

				if (cellDetails[i + 1][j].f == FLT_MAX
					|| cellDetails[i + 1][j].f > fNew) {
					openList.insert(make_pair(
						fNew, make_pair(i + 1, j)));

					cellDetails[i + 1][j].f = fNew;
					cellDetails[i + 1][j].g = gNew;
					cellDetails[i + 1][j].h = hNew;
					cellDetails[i + 1][j].parent_i = i;
					cellDetails[i + 1][j].parent_j = j;
				}
			}
		}

		//East
		if (isValid(i, j + 1) == true) {

			if (isDestination(i, j + 1, dest) == true) {
				cellDetails[i][j + 1].parent_i = i;
				cellDetails[i][j + 1].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}
			else if (closedList[i][j + 1] == false
					&& isUnBlocked(grid, i, j + 1)
							== true) {
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j + 1, dest);
				fNew = gNew + hNew;

				if (cellDetails[i][j + 1].f == FLT_MAX
					|| cellDetails[i][j + 1].f > fNew) {
					openList.insert(make_pair(
						fNew, make_pair(i, j + 1)));

					cellDetails[i][j + 1].f = fNew;
					cellDetails[i][j + 1].g = gNew;
					cellDetails[i][j + 1].h = hNew;
					cellDetails[i][j + 1].parent_i = i;
					cellDetails[i][j + 1].parent_j = j;
				}
			}
		}

		//West
		if (isValid(i, j - 1) == true) {
			if (isDestination(i, j - 1, dest) == true) {
				cellDetails[i][j - 1].parent_i = i;
				cellDetails[i][j - 1].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}

			else if (closedList[i][j - 1] == false
					&& isUnBlocked(grid, i, j - 1)
							== true) {
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j - 1, dest);
				fNew = gNew + hNew;

				if (cellDetails[i][j - 1].f == FLT_MAX
					|| cellDetails[i][j - 1].f > fNew) {
					openList.insert(make_pair(
						fNew, make_pair(i, j - 1)));

					cellDetails[i][j - 1].f = fNew;
					cellDetails[i][j - 1].g = gNew;
					cellDetails[i][j - 1].h = hNew;
					cellDetails[i][j - 1].parent_i = i;
					cellDetails[i][j - 1].parent_j = j;
				}
			}
		}
	}
	
	if (foundDest == false)
		printf("Failed to find the Destination Cell\n");

	return; 
	
}

void timerCallback(const ros::TimerEvent& event){

  waypoint_lat.resize(17);
  waypoint_lat[0] = 42.00244378; 
  waypoint_lat[1] = 41.99978011; 
  waypoint_lat[2] = 42.00036481;
  waypoint_lat[3] = 42.00033951;
  waypoint_lat[4] = 42.00032474;
  waypoint_lat[5] = 42.0013603;
  waypoint_lat[6] = 42.00266004;
  waypoint_lat[7] = 42.00138482;
  waypoint_lat[8] = 42.00139909; 
  waypoint_lat[9] = 42.00323055; 
  waypoint_lat[10] = 42.00272746;
  waypoint_lat[11] = 42.00264799;
  waypoint_lat[12] = 42.00092496;
  waypoint_lat[13] = 42.00256814;
  waypoint_lat[14] = 42.00162285;
  waypoint_lat[15] = 42.00113908;
  waypoint_lat[16] = 42.00242659;
  waypoint_lon.resize(17);
  waypoint_lon[0] = -83.0018947; 
  waypoint_lon[1] = -82.99568642; 
  waypoint_lon[2] = -82.99570677;
  waypoint_lon[3] = -82.99709325;
  waypoint_lon[4] = -82.99787731;
  waypoint_lon[5] = -82.99791196;
  waypoint_lon[6] = -83.0005567;
  waypoint_lon[7] = -82.99652292;
  waypoint_lon[8] = -82.99573927; 
  waypoint_lon[9] = -82.99770135; 
  waypoint_lon[10] = -82.99698865;
  waypoint_lon[11] = -82.99768045;
  waypoint_lon[12] = -82.99711256;
  waypoint_lon[13] = -82.99448124;
  waypoint_lon[14] = -82.9931558;
  waypoint_lon[15] = -82.99513235;
  waypoint_lon[16] = -83.00267861;

  relative_position_waypoint.resize(17);
  for (int k = 0; k < 17; k++) {
    UTMCoords waypoint(LatLon(waypoint_lat[k], waypoint_lon[k], 0.0));
    relative_position_waypoint[k] = waypoint - ref_coords;
  }

  geometry_msgs::PoseStamped current_pose;
  current_pose.pose.position.x = relative_position.x();
  current_pose.pose.position.y = relative_position.y();
  
  //Use i to shorten code
  int i = waypointPath[current_waypoint];
  int currentTurn = waypointTurn[current_waypoint];

  //Heading Calculation
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

  double turnDist;

 if (currentTurn == 1)
  {
    //Right
    turnDist = 9;
  }
  else if(currentTurn == -1){
    //Left
    turnDist = 8;
  }
  else{
    //Straight,
    turnDist = 9;
  }

  cmd_throttle.data = 0.1;
  cmd_brake.data = 0;

  if (dist < turnDist){

    // ROS_INFO("current_waypoint: (%d)",  current_waypoint);
    // ROS_INFO("waypointPath.size: (%lu)",  waypointPath.size());
    if (current_waypoint == (waypointPath.size()-1)){
      cmd_throttle.data = 0;
      cmd_brake.data = 1000;
      ROS_INFO("Braking:");
    }
    else
    {
      atPoint = 1;
      
      if (currentTurn == -1) {
        cmd_steering.data = 4.8; 
        ROS_INFO("Left Turn:");
      }
      else if (currentTurn == 1)
      {
        ROS_INFO("Right Turn:");
        cmd_steering.data = -7.5;
      }
    }
  }
  else{
    if (atPoint == 1) 
    {
      atPoint =0;
      current_waypoint++;
    }
    cmd_steering.data = pathSteeringAngle;
  }

   pub_steering.publish(cmd_steering);
   pub_brake.publish(cmd_brake);
   pub_vel.publish(cmd_throttle);


  ROS_INFO("Calculated Distance: (%f)",  dist);
  ROS_INFO("turnDist : (%f)",  turnDist);
  ROS_INFO("current_waypoint: (%d)",  current_waypoint);
  ROS_INFO("waypointPath.size: (%lu)",  sizeof(waypointPath));
  ROS_INFO("waypointPath.size: (%lu)",  waypointPath.size());
  
}

void recvFix(const sensor_msgs::NavSatFixConstPtr& msg){
  UTMCoords current_coords(*msg);
  veh_lat = msg->latitude;
  veh_lon = msg->longitude;
  relative_position = current_coords - ref_coords;  
}

void recvFix2(const std_msgs::Float64ConstPtr& msg){
  veh_heading = msg->data;
}

void recvCmd(const geometry_msgs::TwistConstPtr& msg)
{
  pathSpeed = msg->linear.x;
  pathAngle = msg->angular.z;
  pathSteeringAngle = gam * atan((L*pathAngle)/pathSpeed);
}

void calcTarget(string start,string end){
  
  //vector<string> StringArray;
  string letterArray[17] = {
    "A",
    "B",
    "C",
    "D",
    "E",
    "F",
    "G",
    "H",
    "I",
    "J",
    "K",
    "L",
    "M",
    "N",
    "O",
    "P",
    "Q",
  };
   int startIdx;  int endIdx;

  for(int s = 0;s<17;s++){
    string letter = letterArray[s];
    if (letter.find(start) != string::npos) {
    //.. found.
    ROS_INFO("Found Start!");
    startIdx = s;
    } 
    if (letter.find(end) != string::npos) {
    ROS_INFO("Found End!");
    endIdx = s;
    }
  }

  startx = locationArray[startIdx][0];
  starty= locationArray[startIdx][1];;
  endx= locationArray[endIdx][0];; 
  endy= locationArray[endIdx][1];;
}


int main(int argc, char** argv){

  calcTarget(StartingIntersection,TargetIntersection);
	Pair src = make_pair(startx, starty);
	Pair dest = make_pair(endx, endy);
	aStarSearch(grid, src, dest);

  ros::init(argc,argv,"gps_sim_nhabben");
  ros::NodeHandle nh;
  ros::Subscriber gps_heading = nh.subscribe("/audibot/gps/fix",1,recvFix);
  ros::Subscriber sub_steer = nh.subscribe("/audibot/cmd_vel",1,recvCmd);
  ros::Subscriber gps_sub = nh.subscribe("/audibot/gps/heading",1,recvFix2);
  ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCallback);

  double ref_lat;
  double ref_lon;

  current_waypoint = 0;

  nh.getParam("/audibot/gps/ref_lat",ref_lat);
  nh.getParam("/audibot/gps/ref_lon",ref_lon);
  
  LatLon ref_coords_lat_lon(ref_lat, ref_lon, 0);
  ref_coords = UTMCoords(ref_coords_lat_lon);
  central_meridian = ref_coords.getCentralMeridian();
  
  pub_vel = nh.advertise<std_msgs::Float64>("/audibot/throttle_cmd", 1);
  pub_steering = nh.advertise<std_msgs::Float64>("/audibot/steering_cmd", 1);
  pub_brake = nh.advertise<std_msgs::Float64>("/audibot/brake_cmd", 1);
  
  ros::spin();
}
