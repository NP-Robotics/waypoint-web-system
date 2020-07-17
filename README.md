# waypoint-web-system
Allows user to save, delete and retrieve waypoints.
Takes in waypoints and tasks from the user and send the path to the mobile base

# Published Topics and Provided Services
## Published Topics
|Topic Name|Message Type|Purpose|
|----------|------------|-------|
|/waypoints|waypoint_msgs::PathTaskArray|Act as a medium to carry the coordinates of waypoints and the tasks at each waypoint from waypoint_to_goal.cpp to path_to_goal_waypoint.cpp for coordinates to publish to the robot. *Note: There is no need for the user to interfere with this topic, publish the waypoints using /web_service/waypoint_sequence service|
## Services
|Service Name|Input Type and Name|Output Type and Name|Purpose|
|---|---|---|---|
|/web_service/add_location|waypoint_msgs/NamePose.srv name x y z w|bool data string success|Saves a waypoint to the database with given name and pose.|
|/web_service/delete_location|waypoint_msgs/Strings.srv name|bool data string success|Remove stored waypoint from database with given name.|
|/web_service/delete_all_locations|std_srvs/SetBool.srv data|bool data string success|Remove all waypoints from database.|
|/web_service/retrieve_location|waypoint_msgs/Waypoint.srv name|bool data string web_system.msg/ID name pose|Retrieve waypoint ffrom database with given name.|
|/web_service/retrieve_all_locations|waypoint_msgs/WaypointsList.srv data|bool data waypoint_msgs/ID[] ID[]|Retreive all waypoints from database in the from of an array.|
|/web_service/waypoint_sequence|waypoint_msgs/WaypointSequence.srv||Obtain a sequence of goal coordinates based on given location sequence and publish to move_base to execute the waypoints and the corresponding task.|
|/web_service/waypoint_sequence|waypoint_msgs/TaskList.srv||Retrieve list of all available tasks.|

# Setup
## Launching move_base node
To use the waypoint system, the move_base node from the Navigation Stack must be executed
## Configuring launch file
Open the launch file in person_tracking_ros/launch/person_tracker.launch

In this line:
```
	<param name="path" type="string" value="/home/hackerman/catkin_ws/src/waypoint-web-system/waypoints/waypoint_system/database/locations.json" />
```
Change `/home/hackerman/catkin_ws/src/waypoint-web-system/waypoints/waypoint_system/database` to your custom directory where your database is stored. Only .json files are supported.
