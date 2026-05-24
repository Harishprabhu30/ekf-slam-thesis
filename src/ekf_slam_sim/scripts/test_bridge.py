import rclpy
from rclpy.node import Node

class BridgeTest(Node):
	def __init__(self):
		super().__init__('bridge_test')
		self.get_logger().info('ROS2-Isaac Sim Bridge Test Node Started')
		
		# Test Subscription to Isaac Sim topics
		from sensor_msgs.msg import Image, LaserScan, Imu
		
		# Monitor for topics
		self.create_timer(1.0, self.check_topics)
		
	def check_topics(self):
		topics = self.get_topic_names_and_types()
		self.get_logger().info(f'Found {len(topics)} topics')
		
		# Filter for Isaac sim topics
		isaac_topics = [t[0] for t in topics if 'isaac' in t[0].lower() or
				'camera' in t[0].lower() or
				'lidar' in t[0].lower()]
		
		if isaac_topics:
			self.get_logger().info('Isaac SIm Topics Found: ')
			for topics in isaac_topics:
				self.get_logger().info(f' - {topic}')

def main():
	rclpy.init()
	node = BridgeTest()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ = '__main__':
	main()
