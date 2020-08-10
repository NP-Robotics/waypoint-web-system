#!/usr/bin/env python

import os
import rospy
import json
from json import JSONDecoder
from waypoint_msgs.srv import NamePose
from waypoint_msgs.srv import Strings
from waypoint_msgs.srv import WaypointsList
from waypoint_msgs.srv import Waypoint
from waypoint_msgs.msg import Locations,ID
from std_srvs.srv import SetBool


class Avatar():
    def __init__(self):
        dir_path = os.path.dirname(os.path.abspath(__file__))
        dir_path = self.walk_up_folder(dir_path)
        self.databaseDirectory = os.path.join(dir_path, 'waypoint_system/database', 'location.json')
        self.database = self.loadDatabase()

        self.addWaypointService = rospy.Service("/web_service/add_location", NamePose, self.callback_addWaypoint)
        self.deleteWaypointService = rospy.Service("/web_service/delete_location", Strings, self.callback_deleteWaypoint)
        self.deleteAllWaypointsService = rospy.Service("/web_service/delete_all_location", SetBool, self.callback_deleteAllWaypoints)
        self.retrieveWaypointService = rospy.Service("/web_service/retrieve_location", Waypoint, self.callback_retrieveWaypoint)
        self.retrieveAllWaypointsService = rospy.Service("/web_service/retrieve_all_location", WaypointsList, self.callback_retrieveAllWaypoints)

        rospy.spin()

    def callback_addWaypoint(self, req):
        if req.name in self.database:
            return False, "Duplicate name!!!"
        else:
            self.database[req.name] = {"x" : req.x, "y" : req.y, "z" : req.z, "w" : req.w, }
            return True, "Succesfully added " + req.name


    def callback_deleteWaypoint(self, req):
        if not req.name in self.database:
            return False, "Waypoint not found!!"
        else:
            self.database.pop(req.name)
            return True, "Succesfully deleted " + req.name
        

    def callback_deleteAllWaypoints(self, req):
        self.database = {}
        return True, "Succesfully deleted all waywaypoints"

    def callback_retrieveWaypoint(self, req):
        if not req.name in self.database:
            return False, "Name not found!!!", ID()
        else:
            waypoint = ID()
            waypoint.name = req.name
            waypoint.pose.x = self.database[req.name]["x"]
            waypoint.pose.y = self.database[req.name]["y"]
            waypoint.pose.w = self.database[req.name]["w"]
            waypoint.pose.z = self.database[req.name]["z"]
            return True, "Successfully retrieved " + req.name, waypoint

    def callback_retrieveAllWaypoints(self, req):
        allWaypoints = []
        location = ID()
        for waypoint in self.database:
            location.name = waypoint
            location.pose.x = self.database[waypoint]["x"]
            location.pose.y = self.database[waypoint]["y"]
            location.pose.z = self.database[waypoint]["z"]
            location.pose.w = self.database[waypoint]["w"]
            allWaypoints.append(location)

        return True, allWaypoints

    def loadDatabase(self):
        try:
            file = open(self.databaseDirectory, 'r+')
            self.database = json.load(file)
            file.close()
            return self.database
        except ValueError:
            return {}

    def dumpDatabase(self):
        file = open(self.databaseDirectory, 'w+')
        file.close()
        file = open(self.databaseDirectory, 'r+')
        json.dump(self.database, file, indent=4, sort_keys=True)
        file.close()
        
    def walk_up_folder(self, path, dir_goal='waypoints'):
    ''' Searches and returns the directory of the waypoint.csv file ''' 

        dir_path = os.path.dirname(path)
        split_path = str.split(dir_path, '/')     
        counter = 0  

        while (split_path[-1] != dir_goal and counter < 20):
            dir_path = os.path.dirname(dir_path)
            split_path = str.split(dir_path, '/')
            counter += 1

        return dir_path

if __name__ == "__main__":
    rospy.init_node("legendOfWebService")
    start = Avatar()
    if rospy.on_shutdown:
        start.dumpDatabase()
