import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import csv
import math
import sys
import json
import subprocess

class RobotPoseLogger(Node):
    def __init__(self):
        super().__init__('robot_pose_odom')
        self.path_csv = 'path/default_path_odom.csv'
        self.map_dir = 'path'
        self.first_pose_saved = False
        self.last_saved_x = None
        self.last_saved_y = None
        self.last_saved_qx = None
        self.last_saved_qy = None
        self.last_saved_qz = None
        self.last_saved_qw = None
        self.pose_data_list = []  # List to store pose data
        
        self.status_publisher = self.create_publisher(
            String,
            '/save_status',
            10
        )
        
        # Publisher for showing list with index and coordinates
        self.show_list_publisher = self.create_publisher(
            String,
            '/show_list',
            10
        )
        
        self.pose_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10)
            
        self.save_file_subscriber = self.create_subscription(
            String,
            '/save_file',
            self.save_file_callback,
            10)
    
    def publish_status(self, success):
        msg = String()
        msg.data = success
        self.status_publisher.publish(msg)
    
    def publish_list(self):
        data_array = []
        
        for idx, data in enumerate(self.pose_data_list):
            data_point = {
                "index": idx,
                "x": round(data[1], 3),  
                "y": round(data[2], 3)   
            }
            data_array.append(data_point)
        
        json_data = json.dumps(data_array)
        
        msg = String()
        msg.data = json_data
        self.show_list_publisher.publish(msg)
    
    def init_csv_file(self, filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'X', 'Y', 'Orientation_X', 'Orientation_Y', 'Orientation_Z', 'Orientation_W'])
    
    def pose_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.qx = msg.pose.pose.orientation.x
        self.qy = msg.pose.pose.orientation.y
        self.qz = msg.pose.pose.orientation.z
        self.qw = msg.pose.pose.orientation.w
        
        if not self.first_pose_saved:
            # Store the first pose data in the list
            self.pose_data_list.append([timestamp, self.x, self.y, self.qx, self.qy, self.qz, self.qw])
            self.first_pose_saved = True
            self.last_saved_x = self.x
            self.last_saved_y = self.y
            self.last_saved_qx = self.qx
            self.last_saved_qy = self.qy
            self.last_saved_qz = self.qz
            self.last_saved_qw = self.qw
            self.publish_status(f'Data point saved to memory')
            # Publish updated list when a point is added
            self.publish_list()
            return
        
        distance = math.sqrt((self.x - self.last_saved_x) ** 2 + (self.y - self.last_saved_y) ** 2)
        if distance >= 1.0:
            # Store pose data in the list if distance is significant
            self.pose_data_list.append([timestamp, self.x, self.y, self.qx, self.qy, self.qz, self.qw])
            self.last_saved_x = self.x
            self.last_saved_y = self.y
            self.last_saved_qx = self.qx
            self.last_saved_qy = self.qy
            self.last_saved_qz = self.qz
            self.last_saved_qw = self.qw
            self.publish_status(f'Data point saved to memory')
            # Publish updated list when a point is added
            self.publish_list()
    
    def save_file_callback(self, msg):
        data = msg.data
        
        # Check if the data is "true" to save all data to file
        if data == "true":
            # Add current position as the last entry
            if self.last_saved_x is not None:
                timestamp = self.get_clock().now().to_msg().sec
                self.pose_data_list.append([timestamp, self.x, self.y, self.qx, self.qy, self.qz, self.qw])
                # Publish updated list with final point
                self.publish_list()
            
            # Initialize the CSV file and write all collected data
            self.init_csv_file(self.path_csv)
            self.write_all_to_csv()
            
            # Save map using subprocess
            map_path = f"{self.map_dir}/map"
            try:
                self.publish_status('Saving map...')
                subprocess.run(
                    ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path, 
                     "--ros-args", "-p", "save_map_timeout:=10000.0"],
                    check=True
                )
                self.publish_status(f'Successfully saved map to {map_path}')
            except subprocess.SubprocessError as e:
                self.publish_status(f'Error saving map: {str(e)}')
            
            self.publish_status(f'Successfully Saved path\nEnd of work!')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        
        # Check if the data is an integer, which means delete that index
        try:
            index = int(data)
            if 0 <= index < len(self.pose_data_list):
                # Delete the item at the specified index
                deleted_item = self.pose_data_list.pop(index)
                x_formatted = f"{deleted_item[1]:.3f}"
                y_formatted = f"{deleted_item[2]:.3f}"
                self.publish_status(f'Deleted index {index}: Position ({x_formatted}, {y_formatted})')
                # Publish updated list after deletion
                self.publish_list()
            else:
                self.publish_status(f'Invalid index: {index}. Valid range: 0-{len(self.pose_data_list)-1}')
        except ValueError:
            # If the data is not "true" or an integer, do nothing
            pass
    
    def write_all_to_csv(self):
        with open(self.path_csv, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for data_row in self.pose_data_list:
                writer.writerow(data_row)
        
        self.publish_status(f'Saved all data to {self.path_csv}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
