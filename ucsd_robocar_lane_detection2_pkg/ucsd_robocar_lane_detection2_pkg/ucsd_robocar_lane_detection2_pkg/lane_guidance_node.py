# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Int32MultiArray, String
from geometry_msgs.msg import Twist
import time
import os
from sensor_msgs.msg import LaserScan
import math

# Node/Topic/Subscriber Parameters
NODE_NAME = 'lane_guidance_node'
LD_NODE_NAME = 'lane_detection_node'
CENTROID_TOPIC_NAME = '/centroid'
ACTUATOR_TOPIC_NAME = '/cmd_vel'
CALIBRATION_NODE_NAME = 'calibration_node'
SUBSCRIBER_TOPIC_NAME = '/scan'
PERSON_DETECTED_TOPIC = '/person_detected'

# Lane Guidance - Path Planning Class      
class PathPlanner(Node):
    # Initializer
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()
        self.centroid_subscriber = self.create_subscription(Float32, CENTROID_TOPIC_NAME, self.controller, 10)
        self.centroid_subscriber # prevent unused variable warning
        
        # Calibration Node Subscription
        self.subscription = self.create_subscription(
            String,
            LD_NODE_NAME,
            self.controller,
            10)
        self.subscription  # prevent unused variable warning

        # Person Detection Subscription
        self.person_subscriber = self.create_subscription(
            Int32,
            PERSON_DETECTED_TOPIC,
            self.person_detector,
            10)
        self.person_subscriber # prevent unused variable warning
        self.person_detected = 0
        
        # Lane Guidance Publishing (Credit to Spring 2022 Team 1)
        self.publisher_ = self.create_publisher(Float32, NODE_NAME , 10)
        self.i = 0.0
        
        # Obstacle Detection Variables
        self.obstacle_detected = 0.0
        self.obstacle_ranges = 0.0
        self.min_angle = 0
        
        # Lidar Subscription (Credit to Spring 2022 Team 1)
        self.sub = self.create_subscription(LaserScan, SUBSCRIBER_TOPIC_NAME, self.detect_obstacle, 10)
        self.sub # prevent unused variable warning
        
        # Lidar Variables (Edit as Wanted)
        self.viewing_angle = 360
        self.max_distance_tolerance = 0.9
        self.min_distance_tolerance = 0.15

        # Camera Subscription
        
        # Default actuator values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 1),
                ('Ki_steering', 0),
                ('Kd_steering', 0),
                ('error_threshold', 0.15),
                ('zero_throttle',0.0),
                ('max_throttle', 0.2),
                ('min_throttle', 0.1),
                ('max_right_steering', 1.0),
                ('max_left_steering', -1.0)
            ])
        self.Kp = self.get_parameter('Kp_steering').value # between [0,1]
        self.Ki = self.get_parameter('Ki_steering').value # between [0,1]
        self.Kd = self.get_parameter('Kd_steering').value # between [0,1]
        self.error_threshold = self.get_parameter('error_threshold').value # between [0,1]
        self.zero_throttle = self.get_parameter('zero_throttle').value # between [-1,1] but should be around 0
        self.max_throttle = self.get_parameter('max_throttle').value # between [-1,1]
        self.min_throttle = self.get_parameter('min_throttle').value # between [-1,1]
        self.max_right_steering = self.get_parameter('max_right_steering').value # between [-1,1]
        self.max_left_steering = self.get_parameter('max_left_steering').value # between [-1,1]

        # initializing PID control
        self.Ts = float(1/20)
        self.ek = 0 # current error
        self.ek_1 = 0 # previous error
        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        self.integral_max = 1E-8
        
        self.get_logger().info(
            f'\nKp_steering: {self.Kp}'
            f'\nKi_steering: {self.Ki}'
            f'\nKd_steering: {self.Kd}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nzero_throttle: {self.zero_throttle}'
            f'\nmax_throttle: {self.max_throttle}'
            f'\nmin_throttle: {self.min_throttle}'
            f'\nmax_right_steering: {self.max_right_steering}'
            f'\nmax_left_steering: {self.max_left_steering}'
        )
        
    # Obstacle Detection Function (Adapted from Spring 2022 Team 1)
    def detect_obstacle(self, data):
        # Setting obstacle_ranges
        self.obstacle_ranges = data.ranges[len(data.ranges) - 1]
        total_number_of_scans = len(data.ranges)
        scans_per_degree = float(total_number_of_scans/self.viewing_angle)
        # Adapt for desired range of view (Edit as Wanted)
        angle_values = [1,5,10,15,20,25,30,35,40,45,50,55,355,350,345,340,335,330,325,320,315,310,305]
        range_values = []
        
        # Loop through angles and scan using lidar
        for angle in angle_values:
            bs = data.ranges[round(angle*scans_per_degree)]
            if self.max_distance_tolerance >= bs >= self.min_distance_tolerance:
                range_values.append(data.ranges[round(angle*scans_per_degree)])
            else:
                range_values.append(float(self.max_distance_tolerance)+1)
            min_distance = min(range_values)

        # Store smallest scanned angle
        min_angle_index = range_values.index(min(range_values))
        self.min_angle = angle_values[min_angle_index]

        # Angle Tolerance Check --> Obstacle Found
        if self.max_distance_tolerance >= abs(min_distance) >= self.min_distance_tolerance:
            # Negative Angle Correction
            if self.min_angle > 180: self.min_angle = self.min_angle - 360
            angle_rad = (self.min_angle * math.pi) / 180
            normalized_angle = math.sin(angle_rad)
            self.obstacle_detected = 1.0  
        # No Obstacle         
        else:
            # Default "Nonsense" Values
            min_distance = -1.0
            normalized_angle = -1.0
            self.obstacle_detected = 0.0  
            
        # Publishing obstacle_detected
        obst = Float32()
        obst.data = float(self.i)
        self.publisher_.publish(obst)
        obstacle_string = str(obst.data)
        self.i = self.obstacle_ranges

    # Get the data from person_detected
    def person_detector(self, msg):
        self.person_detected = msg.data
        self.get_logger().info(f'\n person detected: {msg.data}')
        # Stop the car if person is detected
        if self.person_detected == 1:
            self.twist_cmd.angular.z = 0.0
            self.twist_cmd.linear.x = 0.0
            self.twist_publisher.publish(self.twist_cmd)

    def controller(self, data):
        # Person Detect Case
        if self.person_detected == 1:
            self.person_detected = 0    # reset person_detected and check again
            return
               
        # setting up PID control
        self.ek = data.data

        # Throttle gain scheduling (function of error)
        self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
        throttle_float_raw = ((self.min_throttle - self.max_throttle)  / (1 - self.error_threshold)) * abs(self.ek) + self.inf_throttle
        throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

        # Steering PID terms
        self.proportional_error = self.Kp * self.ek
        self.derivative_error = self.Kd * (self.ek - self.ek_1) / self.Ts
        self.integral_error += self.Ki * self.ek * self.Ts
        self.integral_error = self.clamp(self.integral_error, self.integral_max)
        steering_float_raw = self.proportional_error + self.derivative_error + self.integral_error
        steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)

        # Publish values
        try:
            self.get_logger().info(
            f'checking for obstacles from LIDAR'
            f'\n min_angle: {self.min_angle}'
            f'\n obstacle_result: {self.obstacle_detected}'
            f'\n person detected: {self.person_detected}')
            
            # Obstacle Avoidance Implementation
            if self.obstacle_detected == 1.0:
                # Min_Angle > 0 means object is to the robot's left
                if self.min_angle > 0:
                    # Turn Right
                    self.twist_cmd.angular.z = 0.6
                    self.twist_cmd.linear.x = 0.4
                    self.twist_publisher.publish(self.twist_cmd)
                    time.sleep(0.1)     # Delay
                    # Corrective Left
                    self.twist_cmd.angular.z = -0.6
                    self.twist_cmd.linear.x = 0.4
                    self.twist_publisher.publish(self.twist_cmd)

                # Min_Angle < 0 means object is to the robot's right
                else:
                    # Turn Left
                    self.twist_cmd.angular.z = -0.4
                    self.twist_cmd.linear.x = 0.2 
                    self.twist_publisher.publish(self.twist_cmd)
                    time.sleep(0.1)     # Delay
                    # Corrective Right
                    self.twist_cmd.angular.z = 0.6
                    self.twist_cmd.linear.x = 0.2
                    self.twist_publisher.publish(self.twist_cmd)

            # No Obstacle Detected
            else:
                # Always run 
                #self.twist_cmd.angular.z = 0.0
                #self.twist_cmd.linear.x = 1.0

                # Continue Lane Detection
                self.twist_cmd.angular.z = steering_float
                self.twist_cmd.linear.x = throttle_float
                self.twist_publisher.publish(self.twist_cmd)

                # shift current time and error values to previous values
                self.ek_1 = self.ek

        except KeyboardInterrupt:
            self.twist_cmd.linear.x = self.zero_throttle
            self.twist_publisher.publish(self.twist_cmd)

    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c 

# Main Function
def main(args=None):
    rclpy.init(args=args)
    path_planner_publisher = PathPlanner()
    try:
        rclpy.spin(path_planner_publisher)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        path_planner_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        path_planner_publisher.twist_cmd.linear.x = path_planner_publisher.zero_throttle
        path_planner_publisher.twist_publisher.publish(path_planner_publisher.twist_cmd)
        time.sleep(1)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
        path_planner_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
