import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray

import tkinter as tk
from tkinter import ttk
from tkinter import messagebox

import yaml


class Map2DWaypointPlannerGUI(tk.Tk, Node):
    def __init__(self):
        tk.Tk.__init__(self)
        Node.__init__(self, 'map2d_waypoints_planner')

        self.title("2D Map Waypoints Planner GUI")
        self.resizable(False, False)

        self.instructions1_label = tk.Label(self, 
                text="!!! Click on the rviz to select the waypoints",
                    justify=tk.CENTER)
        self.instructions2_label = tk.Label(self,
                text="!!! Double click on the table to edit the waypoints",
                    justify=tk.CENTER)

        self.map2d_waypoints_label = tk.Label(self, text="Selected waypoints:")
        self.treeview = ttk.Treeview(self)
        self.treeview["columns"]=("Order","Position","Quarternion")
        # Format our columns
        self.treeview.column("#0", width=0, stretch=tk.NO)
        self.treeview.column("Order", anchor=tk.W, width=30)
        self.treeview.column("Position", anchor=tk.W, width=120)
        self.treeview.column("Quarternion", anchor=tk.W, width=120)

        # Create Headings
        self.treeview.heading("#0", text="", anchor=tk.W)
        self.treeview.heading("Order", text="Order", anchor=tk.W)
        self.treeview.heading("Position", text="Position", anchor=tk.W)
        self.treeview.heading("Quarternion", text="Quarternion", anchor=tk.W)
        # self.treeview.bind("<Double-1>", self.on_double_click)

        self.save_name_label = tk.Label(self, text="Map Name:")
        self.save_name_textbox = tk.Entry(self, width=45)
        self.save_name_textbox.insert(0, "map2d_waypoints.yaml")

        self.save_button = tk.Button(self, text="Save",
                                             command=self.save_selected_waypoints)          
        self.clear_button = tk.Button(self, text="Clear",
                                                command=self.clear_selected_waypoints)
        
        self.map2d_waypoints_label.grid(row=0, column=0, columnspan=3, sticky='ew')

        self.instructions1_label.grid(row=0, column=0, columnspan=3, sticky='ew')
        self.instructions2_label.grid(row=1, column=0, columnspan=3, sticky='ew')
        self.map2d_waypoints_label.grid(row=2, column=0, columnspan=3, sticky='w')
        self.treeview.grid(row=3, column=0, columnspan=3, sticky='nsew')
        self.save_name_label.grid(row=4, column=0)
        self.save_name_textbox.grid(row=4, column=1)
        self.save_button.grid(row=5, column=0, sticky='ew')
        self.clear_button.grid(row=5, column=2, sticky='ew')
        
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10)

        self.selected_waypoints_pub = self.create_publisher(MarkerArray, '/selected_waypoints', 10)

        # self.last_pose = Pose()
        self.markers = MarkerArray()
        self.waypoints_counter = 0

        
    def initialpose_callback(self, msg):
        if msg.header.frame_id != "map":
            self.get_logger().warn("Received initialpose message with frame_id not map")
            return
        self.get_logger().info("Received initialpose message with frame_id map")
        last_pose = msg.pose.pose
        self.treeview.insert(parent='', index='end', text='',
                            values=(self.waypoints_counter, 
                                    f'{last_pose.position.x},{last_pose.position.y},{last_pose.position.z}', 
                                    f'{last_pose.orientation.x},{last_pose.orientation.y},{last_pose.orientation.z},{last_pose.orientation.w}'))
        
        # for visiualization
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = self.waypoints_counter
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = last_pose
        marker.scale.x = 1.0
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        self.markers.markers.append(marker)
        self.selected_waypoints_pub.publish(self.markers)
        self.waypoints_counter += 1




    def save_selected_waypoints(self):
        waypoint_data_list = {"waypoints": []}
        logging_file_path = self.save_name_textbox.get()

        with open(logging_file_path, 'w') as f:
            for child in self.treeview.get_children():
                wp_position = self.treeview.item(child)["values"][1].split(',')
                wp_orientation = self.treeview.item(child)["values"][2].split(',')

                waypoint_data = {
                    "position": {
                        "x": wp_position[0],
                        "y": wp_position[1],
                        "z": wp_position[2]
                    },
                    "orientation": {
                        "x": wp_orientation[0],
                        "y": wp_orientation[1],
                        "z": wp_orientation[2],
                        "w": wp_orientation[3]
                    }
                }
                waypoint_data_list["waypoints"].append(waypoint_data)
            yaml.dump(waypoint_data_list, open(logging_file_path, 'w'))
        messagebox.showinfo("Save waypoints", f"Waypoints saved to {logging_file_path}")
        self.get_logger().info("Save selected waypoints")
    
    def clear_selected_waypoints(self):
        self.waypoints_counter = 0
        self.treeview.delete(*self.treeview.get_children())
        for marker in self.markers.markers:
            marker.action = Marker.DELETE
        self.selected_waypoints_pub.publish(self.markers)
        self.markers.markers = []
        self.get_logger().info("Clear selected waypoints")

def main(args=None):
    rclpy.init(args=args)

    i2dmap_gui_logger = Map2DWaypointPlannerGUI()

    while rclpy.ok():
        # we spin both the ROS system and the interface
        rclpy.spin_once(i2dmap_gui_logger, timeout_sec=0.1)  # Run ros2 callbacks
        i2dmap_gui_logger.update()  # Update the tkinter interface

    rclpy.shutdown()


if __name__ == '__main__':
    main()