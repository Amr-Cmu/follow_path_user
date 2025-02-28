import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import csv
import os
import math
import time
import sys

class AMRNav(Node):
    def __init__(self, num):
        super().__init__('amr_nav_nomap')
        self.navigator = BasicNavigator()
        self.status_publisher = self.create_publisher(String, 'nomap_follow_path_status', 10)
        
        self.csv_filename = '/home/thanawat/amr_ws/src/create_path/csv/default_path_odom.csv'
        self.num = int(num)
 
        self.estimate = None 

        self.publish_status("กำลังเตรียมความพร้อม")
        self.navigator.waitUntilNav2Active()

        self.goal_coordinates = self.load_goal_coordinates()
        self.home = self.goal_coordinates[0]
        self.estimate = self.goal_coordinates[-1]

        self.initial_pose()
        self.publish_status("เริ่มการทำงานของ AMR")

    def initial_pose(self):
        if self.goal_coordinates is None:
            self.publish_status('ไม่พบข้อมูล!')
            return
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        
        pose_msg.pose.position.x = self.estimate[0]
        pose_msg.pose.position.y = self.estimate[1]
        pose_msg.pose.orientation.x = self.estimate[2]
        pose_msg.pose.orientation.y = self.estimate[3]
        pose_msg.pose.orientation.z = self.estimate[4]
        pose_msg.pose.orientation.w = self.estimate[5]
        
        self.navigator.setInitialPose(pose_msg)
        self.publish_status('หุ่นยนต์พร้อมทำงานแล้ว')
        time.sleep(3)
        self.start_navigation()

    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_publisher.publish(msg)
        self.get_logger().info(message)

    def load_goal_coordinates(self):
        goal_coordinates = []
        
        if not os.path.exists(self.csv_filename):
            self.get_logger().error(f'ไม่พบไฟล์ {self.csv_filename}!')
            return goal_coordinates

        with open(self.csv_filename, 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)
            for row in reader:
                try:
                    x, y, ox, oy, oz, ow = map(float, row[1:7])
                    goal_coordinates.append((x, y, ox, oy, oz, ow))
                except ValueError:
                    self.get_logger().error(f'ข้อมูลใน CSV ไม่ถูกต้อง: {row}')
        
        return goal_coordinates

    def invert_quaternion(self, oz, ow):
        yaw = math.atan2(2 * oz * ow, 1 - 2 * oz * oz)
        inverted_yaw = yaw + math.pi if yaw < 0 else yaw - math.pi
        cy = math.cos(inverted_yaw * 0.5)
        sy = math.sin(inverted_yaw * 0.5)
        return sy, cy

    def navigate_to_goal(self, goal):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "map"
        pose_goal.header.stamp = self.get_clock().now().to_msg()
        pose_goal.pose.position.x = float(goal[0])
        pose_goal.pose.position.y = float(goal[1])
        pose_goal.pose.orientation.z = float(goal[4])
        pose_goal.pose.orientation.w = float(goal[5])
        
        self.navigator.goToPose(pose_goal)
        
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
        
        result = self.navigator.getResult()
        success = result == TaskResult.SUCCEEDED
        if not success:
            self.get_logger().warn('การนำทางล้มเหลว')
        return success

    def start_navigation(self):
        for cycle in range(self.num):
            self.publish_status(f'เริ่มรอบการนำทางที่ {cycle + 1}/{self.num}')

            # เดินย้อนกลับ
            backward_goals = list(reversed(self.goal_coordinates))
            for i, goal in enumerate(backward_goals, 1):
                self.publish_status(f'รอบที่ {cycle + 1}/{self.num} - กำลังนำทางย้อนกลับ เป้าหมายที่ {i}/{len(backward_goals)}')
                inverted_oz, inverted_ow = self.invert_quaternion(goal[4], goal[5])
                inverted_goal = (goal[0], goal[1], goal[2], goal[3], inverted_oz, inverted_ow)
                success = self.navigate_to_goal(inverted_goal)
                if not success:
                    self.publish_status(f'ไม่สามารถไปถึงเป้าหมายย้อนกลับที่ {i} ได้ ข้ามไปเป้าหมายถัดไป')
                    time.sleep(1)
            
            # เดินไปข้างหน้า
            for i, goal in enumerate(self.goal_coordinates, 1):
                self.publish_status(f'รอบที่ {cycle + 1}/{self.num} - กำลังนำทางไปข้างหน้า เป้าหมายที่ {i}/{len(self.goal_coordinates)}')
                success = self.navigate_to_goal(goal)
                if not success:
                    self.publish_status(f'ไม่สามารถไปถึงเป้าหมายที่ {i} ได้ ข้ามไปเป้าหมายถัดไป')
                    time.sleep(1)

        self.publish_status('การทำงานเสร็จสิ้น!')

def main():
    rclpy.init()
    num = sys.argv[1]
    node = AMRNav(num)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_status('การทำงานถูกยกเลิกโดยผู้ใช้')
    finally:
        node.navigator.cancelTask()
        rclpy.shutdown()

if __name__ == '__main__':
    main()