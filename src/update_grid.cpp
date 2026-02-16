#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <ros/ros.h>
#include <streambuf>
#include <fstream>
#include <map>
#include <cmath>
#include "geometry_msgs/Point.h"
#include "assignment_3/UpdateGrid.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/SetModelState.h"
#include "assignment_3/Sensor.h"
#include "communal_defines.cpp"

// width of each grid (currently set to 1m)
#define GRID_WIDTH 1.0

// ROS service client so sensors have access to bot position
ros::ServiceClient bot_location;
gazebo_msgs::GetModelState srv;

// current grid is always compared with new grid when updating gazebo to move/delete models
int currentGrid[BOARD_H][BOARD_W];

// 2D array of points used as an interface into the positioning system in gazebo
geometry_msgs::Point coordinates[BOARD_H][BOARD_W];

// global access to the spawn, delete, and set services
ros::ServiceClient spawnClient;
ros::ServiceClient deleteClient;
ros::ServiceClient setClient;

// initialise number of spawned survivors and hostiles
int numSurvivors = 0;
int numHostiles = 0;
// initialise check to see if submarine has already been spawned (its moved if already spawned)
bool submarine1Spawned = false;
bool submarine2Spawned = false;
// home directory and model directory
std::string homeDir = getenv("HOME");
std::string modelDir = homeDir + "/catkin_ws/src/3806ict_assignment_3/models/";

// comparison function for Point
struct ComparePoints
{
	bool operator()(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) const
	{
		if (p1.x != p2.x)
			return p1.x < p2.x;
		if (p1.y != p2.y)
			return p1.y < p2.y;
		return p1.z < p2.z;
	}
};

// Dictionary with coordinates as key and survivor/hostile as value
std::map<geometry_msgs::Point, std::string, ComparePoints> objectPositions;

// -- function declarations --
// takes a model type and returns a spawn model request (gazebo_msgs::SpawnModel)
gazebo_msgs::SpawnModel createSpawnRequest(int modelType, geometry_msgs::Point position);
// updates gazebo with the new positions of objects, spawns them if non-existent
bool updateGrid(assignment_3::UpdateGrid::Request &req, assignment_3::UpdateGrid::Response &res);
// simulates something like a sonar sensor which detects objects. Can take variable sensorRange
// and returns an array of detected objects, corresponding to the distance away from the bot's
// current position in east, north, west, south. The bot's current position is taken from
// gazebo get_model_state in an attempt to model a real sensor
bool hostileSensor(assignment_3::Sensor::Request &req, assignment_3::Sensor::Response &res);
// simulates something like a infrared sensor which detects objects. Can take variable sensorRange
// and returns an array of detected objects, corresponding to the distance away from the bot's
// current position in east, north, west, south. The bot's current position is taken from
// gazebo get_model_state in an attempt to model a real sensor
bool survivorSensor(assignment_3::Sensor::Request &req, assignment_3::Sensor::Response &res);

// main
int main(int argc, char **argv)
{
	// initialise the node
	ros::init(argc, argv, "gazebo_object_manager");
	ros::NodeHandle n;
	// creating a client to the get_model_state service, so that the sensors have access to the bot's
	// x, y coordinates at all times for emulation purposes (to try simulate actual sensors)
	bot_location = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	// pretend we're a submarine :)
	srv.request.model_name = "submarine";

	// Initialize current grid to all empty squares
	for (int i = 0; i < BOARD_H; ++i)
		for (int j = 0; j < BOARD_W; ++j)
			currentGrid[i][j] = EMPTY;

	// Initialise coordinates
	for (int i = 0; i < BOARD_H; ++i)
		for (int j = 0; j < BOARD_W; ++j)
		{
			coordinates[i][j].x = i * GRID_WIDTH;
			coordinates[i][j].y = j * GRID_WIDTH;
			coordinates[i][j].z = 0;
		}

	// create client for set_model_state to update model states when moving objects
	setClient = n.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
	// create client for spawning models when generating the initial board
	spawnClient = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	// create client for deleting models when updating/moving objcets
	deleteClient = n.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
	// advertise the hostile sensor emulator for the robot to call when required
	ros::ServiceServer HostileSenService = n.advertiseService("hostile_sensor", hostileSensor);
	// advertise the survivor sensor emulator for the robot to call when required
	ros::ServiceServer SurvivorSenService = n.advertiseService("survivor_sensor", survivorSensor);
	// advertise the update_grid sensor emulator. This is responsible for updating gazebo
	// with new locations of the objects
	ros::ServiceServer updateGridService = n.advertiseService("update_grid", updateGrid);
	ros::spin();
	return 0;
}

gazebo_msgs::SpawnModel createSpawnRequest(int modelType, geometry_msgs::Point position)
{
	// initialise the message for the spawn_sdf_model service
	gazebo_msgs::SpawnModel spawn;
	std::string modelPath;
	if (modelType == SURVIVOR)
	{
		// bowls are used to model survivors
		// Create a unique name from numSurvivors (e.g. bowl0, bowl1)
		spawn.request.model_name = "bowl" + std::to_string(numSurvivors);
		// increment the number of survivors
		numSurvivors++;
		// create path to sdf model in repository
		modelPath = modelDir + "bowl/model.sdf";
	}
	else if (modelType == HOSTILE)
	{
		// cardboard boxes are used to model hostiles
		// Create a unique name from numHostiles (e.g. cardboard_box0, cardboard_box1)
		spawn.request.model_name = "cardboard_box" + std::to_string(numHostiles); // box = hostile
		// increment the number of hostiles
		numHostiles++;
		// create path to sdf model in repository
		modelPath = modelDir + "cardboard_box/model.sdf";
	}
	else if (modelType == SUB)
	{
		// the turtlebot burger from turtlebot_sim package is used to model the submarine
		spawn.request.model_name = "submarine";
		// create path to sdf model in repository
		modelPath = modelDir + "turtlebot3_burger/model.sdf";
	}
	else if (modelType == SUB2)
	{
		// the turtlebot burger from turtlebot_sim package is used to model the submarine
		spawn.request.model_name = "submarine2";
		// create path to sdf model in repository
		modelPath = modelDir + "turtlebot3_burger/model.sdf";
	}

	// open the file containing the sdf model
	std::ifstream t(modelPath);
	if (!t.is_open())
	{
		ROS_WARN("Could not open model file: %s", modelPath.c_str());
		exit(1);
	}
	// read into string for the spawn request
	std::string modelXml((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
	t.close();
	spawn.request.model_xml = modelXml;
	spawn.request.initial_pose.position = position;
	return spawn;
}

bool updateGrid(assignment_3::UpdateGrid::Request &req, assignment_3::UpdateGrid::Response &res)
{
	// Initialise new grid
	std_msgs::Int32MultiArray read_grid = req.grid;
	gazebo_msgs::SetModelState set;
	gazebo_msgs::DeleteModel del;
	gazebo_msgs::SpawnModel spawn;

	// loop through all coordinates in board
	for (int i = 0; i < BOARD_H; ++i)
		for (int j = 0; j < BOARD_W; ++j)
		{
			int oldIndex = currentGrid[i][j];
			int newIndex = read_grid.data[i * BOARD_W + j];
			if (oldIndex != newIndex)
			{
				// Get the corresponding coordinates
				geometry_msgs::Point point = coordinates[i][j];

				// was empty, now a survivor (only occurs in the initial generation)
				if (oldIndex == EMPTY && newIndex == SURVIVOR)
				{
					// Spawn a "Survivor"
					spawn = createSpawnRequest(SURVIVOR, point);
					objectPositions[point] = spawn.request.model_name;
					spawnClient.call(spawn);
				}
				// was a survivor, now the sub. must delete the survivor and replace
				// with the sub
				if (oldIndex == SURVIVOR && newIndex == SUB2)
				{
					// Delete the "Survivor"
					del.request.model_name = objectPositions[point];
					deleteClient.call(del);
					objectPositions.erase(point);
					// Move the "Sub"
					set.request.model_state.model_name = "submarine2";
					set.request.model_state.pose.position = point;
					setClient.call(set);
				}
				// old position was empty, now a hostile, only occurs in spawn condition
				if (oldIndex == EMPTY && newIndex == HOSTILE)
				{
					// Spawn a "Hostile"
					spawn = createSpawnRequest(HOSTILE, point);
					objectPositions[point] = spawn.request.model_name;
					spawnClient.call(spawn);
				}
				// old position was empty or visited, now it holds the sub, so move/spawn the sub
				if ((oldIndex == EMPTY || oldIndex == VISITED) && newIndex == SUB)
				{
					if (submarine1Spawned)
					{
						// Move the "Sub"
						ROS_INFO("Moving sub to pos: (%.0f, %.0f)", point.x, point.y);
						set.request.model_state.model_name = "submarine";
						set.request.model_state.pose.position = point;
						setClient.call(set);
					}
					else if(!submarine1Spawned)
					{
						// Spawn the "Sub"
						spawn = createSpawnRequest(SUB, point);
						objectPositions[point] = spawn.request.model_name;
						spawnClient.call(spawn);
						submarine1Spawned = true;
					}
				}
				if ((oldIndex == EMPTY || oldIndex == VISITED) && newIndex == SUB2)
				{
				if (submarine2Spawned)
				{
					// Move sub2
					ROS_INFO("Moving sub2 to pos: (%.0f, %.0f)", point.x, point.y);
					set.request.model_state.model_name = "submarine2";
					set.request.model_state.pose.position = point;
					setClient.call(set);
				}
				else
				{
					// Spawn sub2
					spawn = createSpawnRequest(SUB2, point);
					objectPositions[point] = spawn.request.model_name;
					spawnClient.call(spawn);
					submarine2Spawned = true;
				}
				}
				// Update current grid
				currentGrid[i][j] = newIndex;
			}
		}

	// Return the updated grid
	res.altered_grid = req.grid;
	return true;
}

bool hostileSensor(assignment_3::Sensor::Request &req, assignment_3::Sensor::Response &res)
{
	// initialise all to false
	res.objectNorth = false;
	res.objectSouth = false;
	res.objectWest = false;
	res.objectEast = false;
	res.objectDetected = false;

	// get current x and y position from gazebo response
	if (!bot_location.call(srv))
	{
		ROS_WARN("Failed to call ROS GetModelState service to get bot location");
		return false;
	}

	// extract x and y from the response. Round as they are given as doubles, we want
	// integers to index into the arrays
	int x = std::round(srv.response.pose.position.x);
	int y = std::round(srv.response.pose.position.y);
	int range = req.sensorRange;

	// Initialize radar arrays
	res.northRadar = std::vector<int32_t>(range, 0);
	res.southRadar = std::vector<int32_t>(range, 0);
	res.eastRadar = std::vector<int32_t>(range, 0);
	res.westRadar = std::vector<int32_t>(range, 0);

	// Check if there are hostiles detected within the sensor range
	for (int i = 1; i <= range; ++i)
	{
		if (x - i >= 0 && currentGrid[x - i][y] == HOSTILE)
		{ // North
			res.objectNorth = true;
			res.northRadar[i - 1] = 1;
		}
		if (x + i < BOARD_H && currentGrid[x + i][y] == HOSTILE)
		{ // South
			res.objectSouth = true;
			res.southRadar[i - 1] = 1;
		}
		if (y - i >= 0 && currentGrid[x][y - i] == HOSTILE)
		{ // West
			res.objectWest = true;
			res.westRadar[i - 1] = 1;
		}
		if (y + i < BOARD_W && currentGrid[x][y + i] == HOSTILE)
		{ // East
			res.objectEast = true;
			res.eastRadar[i - 1] = 1;
		}
	}

	// objectDetected provides easy api to check if anything was detected
	if (res.objectNorth || res.objectEast || res.objectSouth || res.objectWest)
		res.objectDetected = true;
	return true;
}

bool survivorSensor(assignment_3::Sensor::Request &req, assignment_3::Sensor::Response &res)
{
	// initialise all to false
	res.objectNorth = false;
	res.objectSouth = false;
	res.objectWest = false;
	res.objectEast = false;
	res.objectDetected = false;

	// get current x and y position from gazebo response
	if (!bot_location.call(srv))
	{
		ROS_WARN("Failed to call ROS GetModelState service to get bot location");
		return false;
	}

	// extract x and y from the response. Round as they are given as doubles, we want
	// integers to index into the arrays
	int x = std::round(srv.response.pose.position.x);
	int y = std::round(srv.response.pose.position.y);
	int range = req.sensorRange;

	// Initialize radar arrays
	res.northRadar = std::vector<int32_t>(range, 0);
	res.southRadar = std::vector<int32_t>(range, 0);
	res.eastRadar = std::vector<int32_t>(range, 0);
	res.westRadar = std::vector<int32_t>(range, 0);

	// Check if there are survivors detected within the sensor range
	for (int i = 1; i <= range; ++i)
	{
		if (x - i >= 0 && currentGrid[x - i][y] == SURVIVOR)
		{ // North
			res.objectNorth = true;
			res.northRadar[i - 1] = 1;
		}
		if (x + i < BOARD_H && currentGrid[x + i][y] == SURVIVOR)
		{ // South
			res.objectSouth = true;
			res.southRadar[i - 1] = 1;
		}
		if (y - i >= 0 && currentGrid[x][y - i] == SURVIVOR)
		{ // West
			res.objectWest = true;
			res.westRadar[i - 1] = 1;
		}
		if (y + i < BOARD_W && currentGrid[x][y + i] == SURVIVOR)
		{ // East
			res.objectEast = true;
			res.eastRadar[i - 1] = 1;
		}
	}

	// objectDetected provides easy api to check if anything was detected
	if (res.objectNorth || res.objectEast || res.objectSouth || res.objectWest)
		res.objectDetected = true;
	return true;
}