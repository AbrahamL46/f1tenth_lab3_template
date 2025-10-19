import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, 
                                                 self.scan_callback, 10)
        
        self.drive_pub = self.creare_publisher(AckermannDriveStamped, drive_topic, 10)

        # LiDAR metadata (used by get_range)
        self.angle_min = None
        self.angle_increment = None
        self.range_min = None
        self.range_max = None

        # Controller parameters
        self.desired_dist = 1.0     # meters
        self.theta_deg = 50.0       # deg (0, 70]
        self.theta = np.deg2rad(self.theta_deg)
        self.lookahead_L = 0.6      # meters


        # TODO: set PID gains
        self.kp = 1.0 
        self.kd = 0.2
        self.ki = 0.0 

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = None

        # TODO: store any necessary values you think you'll need
        self.max_steer = np.deg2rad(20.0)
        self.i_clip = 2.0

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        if self.angle_min is None or self.angle_increment is None:
            return float('nan')
        
        idx = int(round((angle - self.angle_min) / self.angle_increment))
        idx = max(0, min(idx, len(range_data) - 1))

        #local median over small window to handle NaN/inf
        window = 2
        lo = max(0, idx - window)
        hi = min(len(range_data), idx + window + 1)
        window_vals = [v for v in range_data[lo:hi] if np.isfinite(v)]

        if not window_vals:
            #fallback to single value or range_max
            r = range_data[idx]
            if not np.isfinite(r):
                r = self.range_max if self.range_max is not None else 0.0
            return float(r)
        
        r_med = float(np.median(window_vals))
        #clamp to sensor bounds if known
        if self.range_min is not None and self.range_max is not None:
            r_med = max(self.range_min, min(self.range_max, r_med))
        return r_med
    


    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        
        #desired angles (radians) relative to car +x
        a_ang = np.pi / 2 - self.theta #90 degrees - theta
        b_ang = np.pi / 2 #90 degrees

        a = self.get_range(range_data, a_ang)
        b = self.get_range(range_data, b_ang)

        denom = a * np.sin(self.theta)
        if not np.isfinite(a) or not np.isfinite(b) or abs(denom) < 1e - 6:
            return 0.0
        
        alpha = np.arctan((a * np.cos(self.theta) - b) / denom)
        Dt = b * np.cos(alpha)
        Dt1 = Dt + self.lookahead_L * np.sin(alpha)

        error = dist - Dt1 #positive error => steer left (positive angle)
        return float(error)


    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = 0.0 # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()