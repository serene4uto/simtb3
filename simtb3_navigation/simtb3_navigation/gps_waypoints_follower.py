import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time
import tkinter as tk
from tkinter import filedialog
from simtb3_navigation.utils.gps_utils import latLonYaw2Geopose
from sensor_msgs.msg import NavSatFix


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class GpsWaypointFollowerGUI(tk.Tk, Node):
        
    def __init__(self):
        tk.Tk.__init__(self)
        Node.__init__(self, 'gps_waypoint_follower')
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = None

        self.title("GPS Waypoint Follower GUI")
        self.resizable(False, False)

        self.load_label = tk.Label(self, text="File path:")
        self.load_textbox = tk.Entry(self, width=45)
        self.load_textbox.insert(0, "gps_waypoints.yaml")
        self.select_path_button = tk.Button(self, text="Select",
                                            command=self.select_path)
        
        self.start_button = tk.Button(self, text="Start",
                                            command=self.start_wpf)
        
        self.stop_button = tk.Button(self, text="Stop",
                                            command=self.stop_wpf)
        self.stop_button.config(state=tk.DISABLED)
        
        self.load_label.grid(row=0, column=0, sticky='ew')
        self.load_textbox.grid(row=0, column=1, sticky='ew')
        self.select_path_button.grid(row=0, column=2, sticky='ew')
        self.start_button.grid(row=1, column=0, sticky='ew')
        self.stop_button.grid(row=1, column=2, sticky='ew')

        self.state_check_timer = self.create_timer(1, self.check_nav_state)
        self.state_check_timer.cancel()

        self.selected_wps_pub = self.create_publisher(
            NavSatFix, "/igw_gps_points", 1)
        

    def select_path(self):
        self.load_textbox.delete(0, tk.END)
        self.load_textbox.insert(0, filedialog.askopenfilename())
    
    def start_wpf(self):

        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

        self.wp_parser = YamlWaypointParser(self.load_textbox.get())
        for wp in self.wp_parser.get_wps():
            loaded_wp_msg = NavSatFix()
            loaded_wp_msg.latitude = wp.position.latitude
            loaded_wp_msg.longitude = wp.position.longitude
            self.selected_wps_pub.publish(loaded_wp_msg)
            time.sleep(0.5)

        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        wps = self.wp_parser.get_wps()
        self.navigator.followGpsWaypoints(wps)

        # Start the state check timer
        if self.state_check_timer.is_canceled():
            self.state_check_timer.reset()

    
    def stop_wpf(self):
        self.navigator.cancelTask()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

        # Stop the state check timer    
        if not self.state_check_timer.is_canceled():
            self.state_check_timer.cancel()

    def check_nav_state(self):
        if self.navigator.isTaskComplete():
            print("wps completed successfully")
            self.stop_wpf()


def main(args=None):
    rclpy.init(args=args)

    gps_waypoint_flw = GpsWaypointFollowerGUI()

    while rclpy.ok():
        # we spin both the ROS system and the interface
        rclpy.spin_once(gps_waypoint_flw, timeout_sec=0.1)  # Run ros2 callbacks
        gps_waypoint_flw.update()  # Update the tkinter interface

    rclpy.shutdown()