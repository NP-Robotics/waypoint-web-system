# waypoint-web-system
Allows user to save, delete and retrieve waypoints.
Takes in waypoints and tasks from the user and send the path to the mobile base

# Published Topics and Provided Services
## Published Topics
|Topic Name|Message Type|Purpose|
|----------|------------|-------|
## Services
|Service Name|Input Type and Name|Output Type and Name|Purpose|
|---|---|---|---|
|/web_service/add_location|waypoint_msgs/NamePose.srv name x y z w|bool data string success|Saves a waypoint to the database with given name and pose.|
|/web_service/delete_location|waypoint_msgs/Strings.srv name|bool data string success|Remove stored waypoint from database with given name.|
|/web_service/delete_all_locations|std_srvs/SetBool.srv data|bool data string success|Remove all waypoints from database.|
|/web_service/retrieve_location|waypoint_msgs/Waypoint.srv name|bool data string web_system.msg/ID name pose|Retrieve waypoint ffrom database with given name.|
|/web_service/retrieve_all_locations|waypoint_msgs/WaypointsList.srv data|bool data waypoint_msgs/ID[] ID[]|Retreive all waypoints from database in the from of an array.|

# Setup
## Configuring launch file
Open the launch file in person_tracking_ros/launch/person_tracker.launch

In this line:
```
	<param name="path" type="string" value="/home/hackerman/catkin_ws/src/waypoint-web-system/waypoints/waypoint_system/database/locations.json" />
```
Change `/home/hackerman/catkin_ws/src/waypoint-web-system/waypoints/waypoint_system/database` to your custom directory where your database is stored. Only .json files are supported.
