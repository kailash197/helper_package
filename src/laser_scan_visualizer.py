import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarPolarPlot(Node):
    def __init__(self):
        super().__init__('lidar_polar_plot')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.scan_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_theta_zero_location("N")  # North as 0 degrees
        self.ax.set_theta_direction(1)  # Counterclockwise increment
        plt.ion()
        plt.show()
    
    def scan_callback(self, msg):
        self.ax.clear()
        self.ax.set_theta_zero_location("N")  # Reset North as 0 degrees
        self.ax.set_theta_direction(1)  # Counterclockwise increment
        
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        
        # Clip ranges to be within min and max limits
        ranges = np.clip(ranges, msg.range_min, msg.range_max)
        
        # Convert to polar coordinates (adjusting for north being at 0 degrees)
        angles = angles - np.pi 
        
        self.ax.plot(angles, ranges, 'b', markersize=1, alpha=0.6)
        self.ax.set_title("Live LIDAR Scan", va='bottom')
        plt.draw()
        plt.pause(0.01)

def main():
    rclpy.init()
    node = LidarPolarPlot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
