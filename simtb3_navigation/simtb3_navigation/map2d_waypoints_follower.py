import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import yaml

import tkinter as tk
from tkinter import filedialog

class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of waypoint objects from the yaml file
        """
        pose_wps = []
        for wp in self.wps_dict["waypoints"]:
            pose = Pose()
            pose.position.x = float(wp["position"]["x"])
            pose.position.y = float(wp["position"]["y"])
            pose.position.z = float(wp["position"]["z"])
            pose.orientation.x = float(wp["orientation"]["x"])
            pose.orientation.y = float(wp["orientation"]["y"])
            pose.orientation.z = float(wp["orientation"]["z"])
            pose.orientation.w = float(wp["orientation"]["w"])

            pose_wps.append(pose)
        
        return pose_wps


class Map2DWaypointFollowerGUI(tk.Tk, Node):
    def __init__(self):
        tk.Tk.__init__(self)
        Node.__init__(self, 'map2d_waypoint_follower')
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = None

        self.title("Map2D Waypoint Follower GUI")
        self.resizable(False, False)

        self.load_label = tk.Label(self, text="File path:")
        self.load_textbox = tk.Entry(self, width=45)
        self.load_textbox.insert(0, "map2d_waypoints.yaml")
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

        self.marker_pub = self.create_publisher(
            MarkerArray, "/selected_waypoints", 1)

        self.current_markers = MarkerArray()
        self.pose_list = []

        # self.update_waypoints()

    def update_waypoints(self):
        self.wp_parser = YamlWaypointParser(self.load_textbox.get())
        self.pose_list = self.wp_parser.get_wps()

        # set initial pose
        initial_pose = PoseStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"
        initial_pose.pose = self.pose_list[0]
        self.navigator.setInitialPose(initial_pose)

        if self.current_markers.markers:
            # delete previous markers
            for i in range(len(self.current_markers.markers)):
                self.current_markers.markers[i].action = Marker.DELETE
            self.marker_pub.publish(self.current_markers)
            self.current_markers.markers = []

        # new markers
        for i, pose in enumerate(self.pose_list):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 1.0
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.current_markers.markers.append(marker)

        self.marker_pub.publish(self.current_markers)
    
    def select_path(self):
        self.load_textbox.delete(0, tk.END)
        self.load_textbox.insert(0, filedialog.askopenfilename())
        self.update_waypoints()

    
    def start_wpf(self):
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)


        self.navigator.waitUntilNav2Active(localizer='robot_localization')

        pose_stampeds = []
        for pose in self.pose_list:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose = pose
            pose_stampeds.append(pose_stamped)
        self.navigator.followWaypoints(pose_stampeds)

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

    map2d_waypoint_flw = Map2DWaypointFollowerGUI()

    while rclpy.ok():
        # we spin both the ROS system and the interface
        rclpy.spin_once(map2d_waypoint_flw, timeout_sec=0.1)  # Run ros2 callbacks
        map2d_waypoint_flw.update()  # Update the tkinter interface

    rclpy.shutdown()
