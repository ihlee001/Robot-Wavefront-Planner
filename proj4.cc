#include <libplayerc++/playerc++.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <algorithm>
#include <cmath>
#include <list>
#include <iostream>
#include <fstream>
#include <vector>

//Project 4 for CS521 by Iain Lee
using namespace std;
using namespace PlayerCc;

//Struct to hold vector direction and magnitude
struct Vector{
	double dir;
	double mag;
};

//Struct to hold coordinate information
struct Coordinate{
	float x;
	float y;
};

//Global gridmap that will be used
float gridMap[1000][1000];
int width;
int height;

//Grows the obstacles by x amount on all sides
void grow_obstacles(int x){
	int i, j, k, m, n;
	char inputLine1[80], nextChar;
	int maxVal;

	ifstream inFile("scaled_hospital_section.pnm");
	
	//all places are placed as 0.0
	for (m=0; m<1000; m++)
        for (n=0; n<1000; n++)   
            gridMap[m][n] = 0.0;
	
	/* Read past first line */
    inFile.getline(inputLine1,80);

	/* Read in width, height, maxVal */
    inFile >> width >> height >> maxVal;

	/* Read in map; */
    for (i=0; i<height; i++){    
        for (j=0; j<width; j++) {
			inFile >> nextChar;
			if (!nextChar){
				gridMap[i][j] = 1.0;//Grows obstacles
				for(k = 1; k <= x; k++){
					gridMap[i + k][j] = 1.0;//Below Growth
					gridMap[i + k][j + k] = 1.0;//Below Right Growth			
					gridMap[i][j + k] = 1.0;//Right Growth
					if(j >= k){					
						gridMap[i + k][j - k] = 1.0;//Bottom Left Growth
						gridMap[i][j - k] = 1.0;//Left Growth
						if(i >= k) gridMap[i - k][j - k] = 1.0;//Top Left Growth
					}
					if(i >= k){
						gridMap[i - k][j] = 1.0;//Top Growth
						gridMap[i - k][j + k] = 1.0;//Top Right Growth
					}
				}
			}
		}
	}
    cout << "Map input complete.\n";
	//Writes the first three header lines that are in pnm files
	ofstream outFile("obstacle_growth.pnm");
	outFile << inputLine1 << endl;
	outFile << width << " " << height << endl
		<< maxVal << endl;

	//Writes to output pnm file with the new obstacle growths
	for (i=0; i<height; i++){
		for (j=0; j<width; j++) {
			if (gridMap[i][j] == 1.0) outFile << (char) 0;
			else outFile << (char) -1;
		}
	}
    cout << "Scaled map output to file.\n";
}



//Checks if empty then places a number
void place_number(int y, int x, float number, bool &go, int starty, int startx){
	if(gridMap[y][x] == 0.0) gridMap[y][x] = number + 1;
	if(starty == y && startx == x) go = false;
}

//Wavefront planner
bool wavefront(int starty, int startx, int goaly, int goalx){
	float number = 2;
	gridMap[goaly][goalx] = 2;//Places the goal in the map
	bool go = true;
	int inc = 0;
	//If it finds the start or once iteration amount ends
	while(go && inc < 10000){
		//Searches all of the map to find the current interation then looks around
		for(int i = 0; i < height; i++){
			for(int j = 0; j < width; j++){
				if(gridMap[i][j] == number){
					place_number(i + 1, j, number, go, starty, startx);//Below
					place_number(i + 1, j + 1, number, go, starty, startx);//Below Right
					place_number(i, j + 1, number, go, starty, startx);//Right 
					if(j - 1 >= 0){
						place_number(i + 1, j - 1, number, go, starty, startx);//Bottom Left
						place_number(i, j - 1, number, go, starty, startx);//Left
						if(i - 1 >= 0){
							place_number(i - 1, j - 1, number, go, starty, startx);//Top Left
						}
					}
					if(i - 1 >= 0){
						place_number(i - 1, j, number, go, starty, startx);//Top
						place_number(i - 1, j + 1, number, go, starty, startx);//Top Right
					}
				}
			}
		}
		number++;
		inc++;
	}
	//returns true if it is impossible to find the start
	if(go) return true;
	else return false;
}

//Gets the path from the wavefront planner
void get_path(int y, int x, Coordinate coords[]){
	float current = gridMap[y][x];
	int size = 0;
	//while we haven't reached the goal
	while(current != 2){
		current -= 1;
		Coordinate coord;
		bool happened = true;
		//checks around all sides to find the next step of wave
		if(gridMap[y + 1][x] == current){
			y += 1;//below
			happened = false;
		}
		else if(gridMap[y][x + 1] == current){
			x += 1;//right
			happened = false;
		}
		else if(gridMap[y + 1][x + 1] == current){//below right
			y += 1;
			x += 1;
			happened = false;
		}
		if(x - 1 >= 0 && happened == true){
			if(gridMap[y][x - 1] == current){
				x -= 1;//left
				happened = false;
			}
			else if(gridMap[y + 1][x - 1] == current){//Below left
				y += 1;
				x -= 1;
				happened = false;
			}
		}
		if(y - 1 >= 0 && happened == true){
			if(gridMap[y - 1][x] == current){
				y -= 1; //Top
				happened = false;
			}
			else if(gridMap[y - 1][x + 1] == current){//Top Right
				y -= 1;
				x += 1;
				happened = false;
			}
		}
		if(y - 1 >= 0 && x-1 >= 0 && happened == true){
			if(gridMap[y - 1][x - 1] == current){//Top left
				x -= 1;
				y -= 1;
			}
		}
		coord.y = y;
		coord.x = x;
		coords[size] = coord;
		size++;
	}
}

//Changes from robot cartesian coordinate to pnm coordinates
void cart_to_grid(Coordinate &coord){
	coord.x = (coord.x + 20) * 543 / 40;
	coord.y = abs(coord.y - 9) * 221 / 18;
}

//Changes pnm coordinates to robot cartesian coordinates
void grid_to_cart(Coordinate &coord){
	coord.x = coord.x * 40 / 543 - 20;
	coord.y = (coord.y * 18 / 221 - 9) * (-1);
}

void mark_path(Coordinate coords[], int size){
	cout << "hey\n";
	int i, j, k, m, n;
	char inputLine1[80], nextChar;
	int maxVal;

	ifstream inFile("scaled_hospital_section.pnm");
	
	//all places are placed as 0.0
	for (m=0; m<1000; m++)
        for (n=0; n<1000; n++)   
            gridMap[m][n] = 0.0;
	
	/* Read past first line */
    inFile.getline(inputLine1,80);

	/* Read in width, height, maxVal */
    inFile >> width >> height >> maxVal;
	
	for (i=0; i<height; i++)    
        for (j=0; j<width; j++) {
	  inFile >> nextChar;
	  if (!nextChar)  
	    gridMap[i][j] = 1.0;
	}

	for(int i = 0; i < size; i++){
		cart_to_grid(coords[i]);
		cout << int(coords[i].y) << '\t' << int(coords[i].x) << '\n';
		gridMap[int(coords[i].y)][int(coords[i].x)] = 1;
	}

	//Writes the first three header lines that are in pnm files
	ofstream outFile("mark_path.pnm");
	outFile << inputLine1 << endl;
	outFile << width << " " << height << endl
		<< maxVal << endl;

	//Writes to output pnm file with the marked path
	for (i=0; i<height; i++){
		for (j=0; j<width; j++) {
			if (gridMap[i][j] == 1.0) outFile << (char) 0;
			else outFile << (char) -1;
		}
	}
}

//Changes from yaw to degrees
double yaw_to_degrees(double yaw){
	double yaw_degrees = yaw * 180 / M_PI;
	if(yaw_degrees < 0) yaw_degrees += 360;
	return yaw_degrees;
}

//Changes from degree to yaw
double degrees_to_yaw(double degrees){
	double yaw = degrees * M_PI / 180;
	if(yaw > M_PI) yaw -= 2 * M_PI;
	return yaw;
}

//Sums the vector
Vector vector_sum(Vector v1, Vector v2){
	Vector sum_vector;
	sum_vector.dir = v1.dir;
	sum_vector.mag = v1.mag;
	if(v2.mag == 0){
		return sum_vector;
	}
	double v1_x = v1.mag * cos(v1.dir);
	double v1_y = v1.mag * sin(v1.dir);
	double v2_x = v2.mag * cos(v2.dir);
	double v2_y = v2.mag * sin(v2.dir);
	double v3_x = v1_x + v2_x;
	double v3_y = v1_y + v2_y;
	sum_vector.dir = atan2(v3_y, v3_x);
	sum_vector.mag = sqrt(v3_x * v3_x + v3_y * v3_y);
	return sum_vector;
}

//Behavior that finds the correct yaw to reach to goal
Vector gotogoal(double dx, double dy){
	Vector vector1;
	vector1.dir = atan2(dy, dx);
	vector1.mag = .5;
	return vector1;
}

//Behavior that finds the correct yaw to avoid obstacles
Vector avoid_obstacle(double robotyaw, LaserProxy &lp){
	Vector vector2;
	double left_dist = lp.GetMinLeft();
	double right_dist = lp.GetMinRight();

	double min_dist;
	if(left_dist > right_dist){
		min_dist = right_dist;
		if(robotyaw >= 0) vector2.dir = robotyaw + M_PI/2;
		else vector2.dir = robotyaw - M_PI/2;
	}
	else{
		min_dist = left_dist;
		if(robotyaw >= 0) vector2.dir = robotyaw - M_PI/2;
		else vector2.dir = robotyaw + M_PI/2;
	}

	double D = 0; //Radius of influence	
	
	if(min_dist <= D) vector2.mag = (D - min_dist) / D;
	else vector2.mag = 0;
	return vector2;
}

//Acts on the given yaw and magnitude determines the speed
void act(Position2dProxy &p2dProxy, double mag, double targetyaw, double robotyaw){
	double speed;

	//Determines speed based on distance
	if(mag > .6) speed = .1;
	else speed = .1; 
	
	//If both yaws are postive or negative
	if((targetyaw >= 0 && robotyaw >= 0) || (targetyaw < 0 && robotyaw < 0)){
		if(robotyaw > targetyaw){
			p2dProxy.SetSpeed(speed, -0.5);
		}
		else{
			p2dProxy.SetSpeed(speed, 0.5);
		}
	}
	//If yaws are top and bottom
	else{
		if(robotyaw >= 0){
			double compare = robotyaw - M_PI;
			if(targetyaw >= compare) p2dProxy.SetSpeed(speed, -0.5);
			else p2dProxy.SetSpeed(speed, 0.5); 
		}
		else{
			double compare = robotyaw + M_PI;
			if(targetyaw >= compare) p2dProxy.SetSpeed(speed, -0.5);
			else p2dProxy.SetSpeed(speed, 0.5);
		}
	}
}



//Grabs the necessary data to pilot
void pilot(PlayerClient &robot, Position2dProxy &p2dProxy, LaserProxy &lp, Coordinate coords[], int size)
{
	//Iterates through the given list of coordinates from the wavefront path and makes the robot act
	for(int i = 0; i < size; i++){
		grid_to_cart(coords[i]);//Converts from pnm coords to robot cartesian coords
	}
	for(int i = 0; i < size; i += 5){
		bool go = true;
		while(go){
			
			robot.Read();
			double robotx = p2dProxy.GetXPos();
			double roboty = p2dProxy.GetYPos();
			double robotyaw = p2dProxy.GetYaw();
			double dx = coords[i].x - robotx;
			double dy = coords[i].y - roboty;
			double distance = sqrt(dx * dx + dy * dy);

			Vector vector1 = gotogoal(dx, dy);
			Vector vector2 = avoid_obstacle(robotyaw, lp);
			Vector sum_vector = vector_sum(vector1, vector2);
			act(p2dProxy, sum_vector.mag, sum_vector.dir, robotyaw);

			//goal
			if(distance < 0.15) go = false;
		}		
	}

}

//main is navigator
int main(int argc, char *argv[]){
	grow_obstacles(4);//Grows the obstacles by 4
	Coordinate start;//Start coordinate
	start.x = -10.071;
	start.y = 3.186;
	Coordinate goal;//Goal coordinate
	goal.x = atof(argv[1]);
	goal.y = atof(argv[2]);
	cart_to_grid(start);//translates from cartesian to pnm coordinate
	cart_to_grid(goal);
	
	//Conducts wavefront planning with the goal and start coordinate
	if(wavefront(int(start.y), int(start.x), int(goal.y), int(goal.x))){
		cout << "Sorry this point is not reachable!\n";
		return 0;
	}
	//Grabs the size of the array needed for the number of paths
	int size = gridMap[int(start.y)][int(start.x)] - 2;

	//Gets the paths and places each coordinate into the coordinate array
	Coordinate coords[size];
	get_path(start.y, start.x, coords);

	//Start Robot
	PlayerClient robot("localhost");

	Position2dProxy p2dProxy(&robot, 0);
	LaserProxy lp(&robot, 0);	

	p2dProxy.SetMotorEnable(1);
	p2dProxy.RequestGeom();
	lp.RequestGeom();

	if(argc == 3){
		pilot(robot, p2dProxy, lp, coords, size);
	}	
	else cout << "Incorrect paramters have been given";
	//mark_path(coords, size);//Used to create the marked paths shots
	return 0;
}


