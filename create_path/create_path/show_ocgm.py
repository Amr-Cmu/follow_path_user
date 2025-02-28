import rclpy
from rclpy.node import Node
import json
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion

class MapOdomProcessorNode(Node):
    def __init__(self):
        super().__init__('map_odom_processor_node')

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.web_data_publisher = self.create_publisher(
            String,
            'web_data',
            10
        )
        
        self.latest_map = None
        self.map_metadata = None
        self.latest_odom = None

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        map_data = np.array(msg.data).reshape(height, width)
        
        self.map_metadata = {
            'resolution': msg.info.resolution,
            'width': width,
            'height': height,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y
        }
        
        self.latest_map = map_data.tolist()
        self.publish_data()
        self.get_logger().debug('Processed and published new map data')

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        
        self.latest_odom = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': yaw,
            'linear_velocity': msg.twist.twist.linear.x,
            'angular_velocity': msg.twist.twist.angular.z
        }
        self.publish_data()
        self.get_logger().debug('Processed and published new odom data')

    def publish_data(self):
        if self.latest_map is not None and self.map_metadata is not None:
            data = {
                'type': 'map_update',
                'map_data': self.latest_map,
                'metadata': self.map_metadata
            }
            if self.latest_odom is not None:
                data['odom'] = self.latest_odom
            
            msg = String()
            msg.data = json.dumps(data)
            self.web_data_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    processor = MapOdomProcessorNode()
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()