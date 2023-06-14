import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Int32MultiArray
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import os.path
import time

# Nodes in this program
CALIBRATION_NODE_NAME = 'calibration_node'

# Nodes listening to rosparameters
LG_NODE_NAME = 'lane_guidance_node'
LD_NODE_NAME = 'lane_detection_node'
VESC_NODE_NAME = 'vesc_twist_node'
ADA_NODE_NAME = 'adafruit_twist_node'

# Topics subscribed/published to in this program
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
ACTUATOR_TOPIC_NAME = '/cmd_vel'

IMG_WINDOW_NAME = 'img'
BW_WINDOW_NAME_1 = 'blackAndWhiteImage_gold'
BW_WINDOW_NAME_2 = 'blackAndWhiteImage_silver'
MASK_WINDOW_NAME_1 = 'Golden_Mask'
MASK_WINDOW_NAME_2 = 'Silver_Tear_Mask'
THR_STR_WINDOW_NAME = 'throttle_and_steering'

cv2.namedWindow(IMG_WINDOW_NAME)
cv2.namedWindow(BW_WINDOW_NAME_1)
cv2.namedWindow(BW_WINDOW_NAME_2)
cv2.namedWindow(MASK_WINDOW_NAME_1)
cv2.namedWindow(MASK_WINDOW_NAME_2)
cv2.namedWindow(THR_STR_WINDOW_NAME)

#
def callback(x):
    pass


def slider_to_normalized(slider_input):
    input_start = 0
    input_end = 2000
    output_start = -1
    output_end = 1
    normalized_output = float(output_start + (slider_input - input_start) * (
            (output_end - output_start) / (input_end - input_start)))
    return normalized_output


lowH_silver = 0
highH_silver = 179
lowS_silver = 0
highS_silver = 255
lowV_silver = 0
highV_silver = 255

lowH_gold = 0
highH_gold = 179
lowS_gold = 0
highS_gold = 255
lowV_gold = 0
highV_gold = 255

glow_gold = 0
ghigh_gold = 255

not_inverted_gold = 0
inverted_gold = 1

blur_kernal_min_gold = 1
blur_kernal_max_gold = 50

dilation_min_gold = 1
dilation_max_gold = 10

glow_silver = 0
ghigh_silver = 255

not_inverted_silver = 0
inverted_silver = 1

blur_kernal_min_silver = 1
blur_kernal_max_silver = 50

dilation_min_silver = 1
dilation_max_silver = 10

min_width = 10
max_width = 500

max_number_of_lines = 100
max_error_threshold = 100
default_error_threshold = 20

min_camera_centerline = 50
max_camera_centerline = 100

min_frame_height = 1
max_frame_height = 100
default_frame_width = 100
max_frame_width = 100
default_min_rows = 50
max_rows = 100
default_min_offset = 50
max_offset = 100

steer_left = 0
steer_straight = 1000
steer_right = 2000

Kp_steering_max = 100
Kp_steering_default = 100
Ki_steering_max = 100
Ki_steering_default = 100
Kd_steering_max = 100
Kd_steering_default = 100

throttle_reverse = 0
throttle_neutral = 1000
throttle_forward = 2000

zero_throttle_mode = 0
max_throttle_mode = 1
min_throttle_mode = 2

max_left_steering_mode = 0
straight_steering_mode = 1
max_right_steering_mode = 2

max_rpm = 20000
zero_rpm = 0

steering_polarity_normal = 1
steering_polarity_reversed = 0
throttle_polarity_normal = 1
throttle_polarity_reversed = 0

# CV filtering 
cv2.createTrackbar('lowH_gold', MASK_WINDOW_NAME_1 , lowH_gold, highH_gold, callback)
cv2.createTrackbar('highH_gold', MASK_WINDOW_NAME_1 , highH_gold, highH_gold, callback)
cv2.createTrackbar('lowS_gold', MASK_WINDOW_NAME_1 , lowS_gold, highS_gold, callback)
cv2.createTrackbar('highS_gold', MASK_WINDOW_NAME_1 , highS_gold, highS_gold, callback)
cv2.createTrackbar('lowV_gold', MASK_WINDOW_NAME_1 , lowV_gold, highV_gold, callback)
cv2.createTrackbar('highV_gold', MASK_WINDOW_NAME_1 , highV_gold, highV_gold, callback)

cv2.createTrackbar('lowH_silver', MASK_WINDOW_NAME_2 , lowH_silver, highH_silver, callback)
cv2.createTrackbar('highH_silver', MASK_WINDOW_NAME_2 , highH_silver, highH_silver, callback)
cv2.createTrackbar('lowS_silver', MASK_WINDOW_NAME_2 , lowS_silver, highS_silver, callback)
cv2.createTrackbar('highS_silver', MASK_WINDOW_NAME_2 , highS_silver, highS_silver, callback)
cv2.createTrackbar('lowV_silver', MASK_WINDOW_NAME_2 , lowV_silver, highV_silver, callback)
cv2.createTrackbar('highV_silver', MASK_WINDOW_NAME_2 , highV_silver, highV_silver, callback)


cv2.createTrackbar('gray_lower_gold', BW_WINDOW_NAME_1 , glow_gold, ghigh_gold, callback)
cv2.createTrackbar('Inverted_filter_gold', BW_WINDOW_NAME_1 , not_inverted_gold, inverted_gold, callback)
cv2.createTrackbar('kernal_size_gold', BW_WINDOW_NAME_1 , blur_kernal_min_gold, blur_kernal_max_gold, callback)
cv2.createTrackbar('erosion_itterations_gold', BW_WINDOW_NAME_1 , dilation_min_gold, dilation_max_gold, callback)
cv2.createTrackbar('dilation_itterations_gold', BW_WINDOW_NAME_1 , dilation_min_gold, dilation_max_gold, callback)

cv2.createTrackbar('gray_lower_silver', BW_WINDOW_NAME_2 , glow_silver, ghigh_silver, callback)
cv2.createTrackbar('Inverted_filter_silver', BW_WINDOW_NAME_2 , not_inverted_silver, inverted_silver, callback)
cv2.createTrackbar('kernal_size_silver', BW_WINDOW_NAME_2 , blur_kernal_min_silver, blur_kernal_max_silver, callback)
cv2.createTrackbar('erosion_itterations_silver', BW_WINDOW_NAME_2 , dilation_min_silver, dilation_max_silver, callback)
cv2.createTrackbar('dilation_itterations_silver', BW_WINDOW_NAME_2 , dilation_min_silver, dilation_max_silver, callback)

# Tracking constraints
cv2.createTrackbar('min_width', IMG_WINDOW_NAME, min_width, max_width, callback)
cv2.createTrackbar('max_width', IMG_WINDOW_NAME, max_width, max_width, callback)
cv2.createTrackbar('number_of_lines', IMG_WINDOW_NAME, max_number_of_lines, max_number_of_lines, callback)
cv2.createTrackbar('error_threshold', IMG_WINDOW_NAME, default_error_threshold, max_error_threshold, callback)
cv2.createTrackbar('camera_centerline', IMG_WINDOW_NAME, min_camera_centerline, max_camera_centerline, callback)

# Image dims
cv2.createTrackbar('frame_width', IMG_WINDOW_NAME, default_frame_width, max_frame_width, callback)
cv2.createTrackbar('rows_to_watch', IMG_WINDOW_NAME, default_min_rows, max_rows, callback)
cv2.createTrackbar('rows_offset', IMG_WINDOW_NAME, default_min_offset, max_offset, callback)

# Steering
cv2.createTrackbar('Kp_steering', THR_STR_WINDOW_NAME, Kp_steering_default, Kp_steering_max, callback)
cv2.createTrackbar('Ki_steering', THR_STR_WINDOW_NAME, Ki_steering_default, Ki_steering_max, callback)
cv2.createTrackbar('Kd_steering', THR_STR_WINDOW_NAME, Kd_steering_default, Kd_steering_max, callback)
cv2.createTrackbar('Steering_mode', THR_STR_WINDOW_NAME, straight_steering_mode, max_right_steering_mode, callback)
cv2.createTrackbar('Steering_value', THR_STR_WINDOW_NAME, steer_straight, steer_right, callback)

# Throttle
cv2.createTrackbar('Throttle_mode', THR_STR_WINDOW_NAME, zero_throttle_mode, min_throttle_mode, callback)
cv2.createTrackbar('Throttle_value', THR_STR_WINDOW_NAME, throttle_neutral, throttle_forward, callback)
cv2.createTrackbar('max_rpm', THR_STR_WINDOW_NAME, max_rpm, max_rpm, callback)

# Actuators polarity
cv2.createTrackbar('steering_polarity', THR_STR_WINDOW_NAME, steering_polarity_normal, steering_polarity_normal, callback)
cv2.createTrackbar('throttle_polarity', THR_STR_WINDOW_NAME, throttle_polarity_normal, throttle_polarity_normal, callback)

# Test motor control
cv2.createTrackbar('test_motor_control', THR_STR_WINDOW_NAME, 0, 1, callback)


class Calibration(Node):
    def __init__(self):
        super().__init__(CALIBRATION_NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()
        
        '''
        #My personal attempt at making this publish a specific value
        self.publisher_ = self.create_publisher(Float32, CALIBRATION_NODE_NAME, 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.p = 0.0
        '''
        
        self.camera_subscriber = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.live_calibration_values, 10)
        self.camera_subscriber
        self.bridge = CvBridge()
        self.ek = 0 # initial error for centroid
        self.ek_1 = 0
        self.Ts = float(1/20) # sample period for motors
        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        self.integral_max = 1E-8

        # declare parameters
        

        self.declare_parameters(
            namespace='',
            parameters=[
                ('Hue_low_gold',1),
                ('Hue_high_gold',1),
                ('Saturation_low_gold',1),
                ('Saturation_high_gold',1),
                ('Value_low_gold',1),
                ('Value_high_gold',1),
                ('Hue_low_silver',1),
                ('Hue_high_silver',1),
                ('Saturation_low_silver',1),
                ('Saturation_high_silver',1),
                ('Value_low_silver',1),
                ('Value_high_silver',1),
                ('gray_lower_gold',1),
                ('gray_lower_silver',1),
                ('Inverted_filter_gold',0),
                ('Inverted_filter_silver',0),
                ('kernal_size_gold',1),
                ('kernal_size_silver',1),
                ('erosion_itterations_gold',1),
                ('erosion_itterations_silver',1),
                ('dilation_itterations_gold',1),
                ('dilation_itterations_silver',1),
                ('Width_min',1),
                ('Width_max',1),
                ('number_of_lines',1),
                ('camera_start_height',1),
                ('camera_bottom_height',1),
                ('camera_left_width',1),
                ('camera_right_width',1),
                ('camera_centerline',0.5),
                ('error_threshold', 0.15),
                ('Kp_steering', 1.0),
                ('Ki_steering', 0.0),
                ('Kd_steering', 0.0),
                ('max_right_steering', 1.0),
                ('straight_steering', 0.0),
                ('max_left_steering', -1.0),
                ('zero_throttle',0.0),
                ('max_throttle', 0.2),
                ('min_throttle', 0.1),
                ('max_rpm',1000),
                ('steering_polarity',1),
                ('throttle_polarity',1)
            ])

        # Get previously set params
        self.Hue_low_gold = self.get_parameter('Hue_low_gold').value
        self.Hue_high_gold = self.get_parameter('Hue_high_gold').value
        self.Saturation_low_gold = self.get_parameter('Saturation_low_gold').value
        self.Saturation_high_gold = self.get_parameter('Saturation_high_gold').value
        self.Value_low_gold = self.get_parameter('Value_low_gold').value
        self.Value_high_gold = self.get_parameter('Value_high_gold').value
        
        self.Hue_low_silver = self.get_parameter('Hue_low_silver').value
        self.Hue_high_silver = self.get_parameter('Hue_high_silver').value
        self.Saturation_low_silver = self.get_parameter('Saturation_low_silver').value
        self.Saturation_high_silver = self.get_parameter('Saturation_high_silver').value
        self.Value_low_silver = self.get_parameter('Value_low_silver').value
        self.Value_high_silver = self.get_parameter('Value_high_silver').value
        
        self.gray_lower_gold = self.get_parameter('gray_lower_gold').value
        self.Inverted_filter_gold = self.get_parameter('Inverted_filter_gold').value
        self.kernal_size_gold = self.get_parameter('kernal_size_gold').value
        self.erosion_itterations_gold = self.get_parameter('erosion_itterations_gold').value
        self.dilation_itterations_gold = self.get_parameter('dilation_itterations_gold').value
        
        self.gray_lower_silver = self.get_parameter('gray_lower_silver').value
        self.Inverted_filter_silver = self.get_parameter('Inverted_filter_silver').value
        self.kernal_size_silver = self.get_parameter('kernal_size_silver').value
        self.erosion_itterations_silver = self.get_parameter('erosion_itterations_silver').value
        self.dilation_itterations_silver = self.get_parameter('dilation_itterations_silver').value
        
        self.number_of_lines = self.get_parameter('number_of_lines').value
        self.error_threshold = self.get_parameter('error_threshold').value
        self.min_width = self.get_parameter('Width_min').value
        self.max_width = self.get_parameter('Width_max').value
        self.camera_start_height = self.get_parameter('camera_start_height').value
        self.camera_bottom_height = self.get_parameter('camera_bottom_height').value
        self.camera_left_width = self.get_parameter('camera_left_width').value
        self.camera_right_width = self.get_parameter('camera_right_width').value
        self.camera_centerline = self.get_parameter('camera_centerline').value
        self.error_threshold = self.get_parameter('error_threshold').value 

        self.Kp_steering = self.get_parameter('Kp_steering').value
        self.Ki_steering = self.get_parameter('Ki_steering').value
        self.Kd_steering = self.get_parameter('Kd_steering').value
        self.max_right_steering = self.get_parameter('max_right_steering').value
        self.straight_steering = self.get_parameter('straight_steering').value
        self.max_left_steering = self.get_parameter('max_left_steering').value
        
        self.zero_throttle = self.get_parameter('zero_throttle').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.min_throttle = self.get_parameter('min_throttle').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.steering_polarity = self.get_parameter('steering_polarity').value
        self.throttle_polarity = self.get_parameter('throttle_polarity').value

        self.get_logger().info(
            f'\nHue_low_gold: {self.Hue_low_gold}'
            f'\nHue_high_gold: {self.Hue_high_gold}'
            f'\nSaturation_low_gold: {self.Saturation_low_gold}'
            f'\nSaturation_high_gold: {self.Saturation_high_gold}'
            f'\nValue_low_gold: {self.Value_low_gold}'
            f'\nValue_high_gold: {self.Value_high_gold}'
            
            f'\nHue_low_silver: {self.Hue_low_silver}'
            f'\nHue_high_silver: {self.Hue_high_silver}'
            f'\nSaturation_low_silver: {self.Saturation_low_silver}'
            f'\nSaturation_high_silver: {self.Saturation_high_silver}'
            f'\nValue_low_silver: {self.Value_low_silver}'
            f'\nValue_high_silver: {self.Value_high_silver}'
            
            f'\ngray_lower_gold: {self.gray_lower_gold}'
            f'\nInverted_filter_gold: {self.Inverted_filter_gold}'
            f'\nkernal_size_gold: {self.kernal_size_gold}'
            f'\nerosion_itterations_gold: {self.erosion_itterations_gold}'
            f'\ndilation_itterations_gold: {self.dilation_itterations_gold}'
            
            f'\ngray_lower_silver: {self.gray_lower_silver}'
            f'\nInverted_filter_silver: {self.Inverted_filter_silver}'
            f'\nkernal_size_silver: {self.kernal_size_silver}'
            f'\nerosion_itterations_silver: {self.erosion_itterations_silver}'
            f'\ndilation_itterations_silver: {self.dilation_itterations_silver}'
            
            f'\nnumber_of_lines: {self.number_of_lines}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nmin_width: {self.min_width}'
            f'\nmax_width: {self.max_width}'
            f'\ncamera_centerline: {self.camera_centerline}')

        try:
            # Set trackbars to previously saved config
            cv2.setTrackbarPos('lowH_gold', MASK_WINDOW_NAME_1, self.Hue_low_gold)
            cv2.setTrackbarPos('highH_gold', MASK_WINDOW_NAME_1, self.Hue_high_gold)
            cv2.setTrackbarPos('lowS_gold', MASK_WINDOW_NAME_1, self.Saturation_low_gold)
            cv2.setTrackbarPos('highS_gold', MASK_WINDOW_NAME_1, self.Saturation_high_gold)
            cv2.setTrackbarPos('lowV_gold', MASK_WINDOW_NAME_1, self.Value_low_gold)
            cv2.setTrackbarPos('highV_gold', MASK_WINDOW_NAME_1, self.Value_high_gold)
            
            cv2.setTrackbarPos('lowH_silver', MASK_WINDOW_NAME_2, self.Hue_low_silver)
            cv2.setTrackbarPos('highH_silver', MASK_WINDOW_NAME_2, self.Hue_high_silver)
            cv2.setTrackbarPos('lowS_silver', MASK_WINDOW_NAME_2, self.Saturation_low_silver)
            cv2.setTrackbarPos('highS_silver', MASK_WINDOW_NAME_2, self.Saturation_high_silver)
            cv2.setTrackbarPos('lowV_silver', MASK_WINDOW_NAME_2, self.Value_low_silver)
            cv2.setTrackbarPos('highV_silver', MASK_WINDOW_NAME_2, self.Value_high_silver)
            
            cv2.setTrackbarPos('gray_lower_gold', BW_WINDOW_NAME_1, self.gray_lower_gold)
            cv2.setTrackbarPos('Inverted_filter_gold', BW_WINDOW_NAME_1, self.Inverted_filter_gold)
            cv2.setTrackbarPos('kernal_size_gold', BW_WINDOW_NAME_1, self.kernal_size_gold)
            cv2.setTrackbarPos('erosion_itterations_gold', BW_WINDOW_NAME_1, self.erosion_itterations_gold)
            cv2.setTrackbarPos('dilation_itterations_gold', BW_WINDOW_NAME_1, self.dilation_itterations_gold)
            
            cv2.setTrackbarPos('gray_lower_silver', BW_WINDOW_NAME_2, self.gray_lower_silver)
            cv2.setTrackbarPos('Inverted_filter_silver', BW_WINDOW_NAME_2, self.Inverted_filter_silver)
            cv2.setTrackbarPos('kernal_size_silver', BW_WINDOW_NAME_2, self.kernal_size_silver)
            cv2.setTrackbarPos('erosion_itterations_silver', BW_WINDOW_NAME_2, self.erosion_itterations_silver)
            cv2.setTrackbarPos('dilation_itterations_silver', BW_WINDOW_NAME_2, self.dilation_itterations_silver)
            
            cv2.setTrackbarPos('min_width', IMG_WINDOW_NAME, self.min_width)
            cv2.setTrackbarPos('max_width', IMG_WINDOW_NAME, self.max_width)
            cv2.setTrackbarPos('number_of_lines', IMG_WINDOW_NAME, self.number_of_lines)
            cv2.setTrackbarPos('error_threshold', IMG_WINDOW_NAME, int(self.error_threshold*100))
            cv2.setTrackbarPos('camera_centerline', IMG_WINDOW_NAME, int(self.error_threshold*100))

            ## To do:
            # self.camera_start_height = self.get_parameter('camera_start_height').value
            # self.camera_bottom_height = self.get_parameter('camera_bottom_height').value
            # self.camera_left_width = self.get_parameter('camera_left_width').value
            # self.camera_right_width = self.get_parameter('camera_right_width').value

            cv2.setTrackbarPos('Kp_steering', THR_STR_WINDOW_NAME, self.Kp_steering)
            cv2.setTrackbarPos('Ki_steering', THR_STR_WINDOW_NAME, self.Ki_steering)
            cv2.setTrackbarPos('Kd_steering', THR_STR_WINDOW_NAME, self.Kd_steering)
            cv2.setTrackbarPos('max_rpm', THR_STR_WINDOW_NAME, self.max_rpm)
            cv2.setTrackbarPos('steering_polarity', THR_STR_WINDOW_NAME, self.steering_polarity)
            cv2.setTrackbarPos('throttle_polarity', THR_STR_WINDOW_NAME, self.throttle_polarity)
        except TypeError:
            pass


    def live_calibration_values(self, data):
        
        # get trackbar positions
        self.Hue_low_gold = cv2.getTrackbarPos('lowH_gold', MASK_WINDOW_NAME_1)
        self.Hue_high_gold = cv2.getTrackbarPos('highH_gold', MASK_WINDOW_NAME_1)
        self.Saturation_low_gold = cv2.getTrackbarPos('lowS_gold', MASK_WINDOW_NAME_1)
        self.Saturation_high_gold = cv2.getTrackbarPos('highS_gold', MASK_WINDOW_NAME_1)
        self.Value_low_gold = cv2.getTrackbarPos('lowV_gold', MASK_WINDOW_NAME_1)
        self.Value_high_gold = cv2.getTrackbarPos('highV_gold', MASK_WINDOW_NAME_1)
        
        self.Hue_low_silver = cv2.getTrackbarPos('lowH_silver', MASK_WINDOW_NAME_2)
        self.Hue_high_silver = cv2.getTrackbarPos('highH_silver', MASK_WINDOW_NAME_2)
        self.Saturation_low_silver = cv2.getTrackbarPos('lowS_silver', MASK_WINDOW_NAME_2)
        self.Saturation_high_silver = cv2.getTrackbarPos('highS_silver', MASK_WINDOW_NAME_2)
        self.Value_low_silver = cv2.getTrackbarPos('lowV_silver', MASK_WINDOW_NAME_2)
        self.Value_high_silver = cv2.getTrackbarPos('highV_silver', MASK_WINDOW_NAME_2)
        
        self.gray_lower_gold = cv2.getTrackbarPos('gray_lower_gold', BW_WINDOW_NAME_1)
        self.Inverted_filter_gold = cv2.getTrackbarPos('Inverted_filter_gold', BW_WINDOW_NAME_1)
        self.kernal_size_gold = cv2.getTrackbarPos('kernal_size_gold', BW_WINDOW_NAME_1)
        self.erosion_itterations_gold = cv2.getTrackbarPos('erosion_itterations_gold', BW_WINDOW_NAME_1)
        self.dilation_itterations_gold = cv2.getTrackbarPos('dilation_itterations_gold', BW_WINDOW_NAME_1)
        
        self.gray_lower_silver = cv2.getTrackbarPos('gray_lower_silver', BW_WINDOW_NAME_2)
        self.Inverted_filter_silver = cv2.getTrackbarPos('Inverted_filter_silver', BW_WINDOW_NAME_2)
        self.kernal_size_silver = cv2.getTrackbarPos('kernal_size_silver', BW_WINDOW_NAME_2)
        self.erosion_itterations_silver = cv2.getTrackbarPos('erosion_itterations_silver', BW_WINDOW_NAME_2)
        self.dilation_itterations_silver = cv2.getTrackbarPos('dilation_itterations_silver', BW_WINDOW_NAME_2)
        
        self.min_width = cv2.getTrackbarPos('min_width', IMG_WINDOW_NAME)
        self.max_width = cv2.getTrackbarPos('max_width', IMG_WINDOW_NAME)
        self.number_of_lines = cv2.getTrackbarPos('number_of_lines', IMG_WINDOW_NAME)
        crop_width_percent = cv2.getTrackbarPos('frame_width', IMG_WINDOW_NAME)
        rows_to_watch_percent = cv2.getTrackbarPos('rows_to_watch', IMG_WINDOW_NAME)
        rows_offset_percent = cv2.getTrackbarPos('rows_offset', IMG_WINDOW_NAME)
        self.error_threshold = float(cv2.getTrackbarPos('error_threshold', IMG_WINDOW_NAME)/100)
        self.camera_centerline = float(cv2.getTrackbarPos('camera_centerline', IMG_WINDOW_NAME)/100)
        
        # Motor parameters
        steering_mode = cv2.getTrackbarPos('Steering_mode', THR_STR_WINDOW_NAME)
        steer_input = cv2.getTrackbarPos('Steering_value', THR_STR_WINDOW_NAME)
        self.Kp_steering = float(cv2.getTrackbarPos('Kp_steering', THR_STR_WINDOW_NAME)/100)
        self.Ki_steering = float(cv2.getTrackbarPos('Ki_steering', THR_STR_WINDOW_NAME)/1000000)
        self.Kd_steering = float(cv2.getTrackbarPos('Kd_steering', THR_STR_WINDOW_NAME)/1000000)
        throttle_mode = cv2.getTrackbarPos('Throttle_mode', THR_STR_WINDOW_NAME)
        throttle_input = cv2.getTrackbarPos('Throttle_value', THR_STR_WINDOW_NAME)
        self.max_rpm = int(cv2.getTrackbarPos('max_rpm', THR_STR_WINDOW_NAME))
        
        # Motor polarities
        steering_pol_slider = int(cv2.getTrackbarPos('steering_polarity', THR_STR_WINDOW_NAME))
        throttle_pol_slider = int(cv2.getTrackbarPos('throttle_polarity', THR_STR_WINDOW_NAME))

        # Test Motor parameters
        test_motor_control = int(cv2.getTrackbarPos('test_motor_control', THR_STR_WINDOW_NAME))
        
        if steering_pol_slider == 0:
            self.steering_polarity = int(-1)
        else:
            self.steering_polarity = int(1)
        if throttle_pol_slider == 0:
            self.throttle_polarity = int(-1)
        else:
            self.throttle_polarity = int(1)
        
        # Setting throttle limits based on mode
        if throttle_mode == 0:
            self.zero_throttle = slider_to_normalized(throttle_input)
        elif throttle_mode == 1:
            self.max_throttle = slider_to_normalized(throttle_input)
        elif throttle_mode == 2:
            self.min_throttle = slider_to_normalized(throttle_input)

        # Setting steering limits based on mode
        if steering_mode == 0:
            self.max_left_steering = slider_to_normalized(steer_input)
        elif steering_mode == 1:
            self.straight_steering = slider_to_normalized(steer_input)
        elif steering_mode == 2:
            self.max_right_steering = slider_to_normalized(steer_input)

        if test_motor_control==1:
            # Throttle gain scheduling (function of error)
            self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
            throttle_float_raw = (self.min_throttle - self.max_throttle) * abs(self.ek) + self.inf_throttle
            throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

            # Steering PID terms
            self.proportional_error = self.Kp_steering * self.ek
            self.derivative_error = self.Kd_steering * (self.ek - self.ek_1) / self.Ts
            self.integral_error += self.Ki_steering * self.ek * self.Ts
            self.integral_error = self.clamp(self.integral_error, self.integral_max)
            steering_float_raw = self.proportional_error + self.derivative_error + self.integral_error
            steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)

            # Publish actuator control signals
            self.twist_cmd.angular.z = self.steering_polarity * steering_float + self.straight_steering
            self.twist_cmd.linear.x = self.throttle_polarity * throttle_float
            self.twist_publisher.publish(self.twist_cmd)

            # shift error term
            self.ek_1 = self.ek

        elif test_motor_control==0:
            self.ek_1 = 0
            self.integral_error = 0
            # Publish constrained actuator values
            self.twist_cmd.angular.z = self.steering_polarity * slider_to_normalized(steer_input)
            self.twist_cmd.linear.x = self.throttle_polarity * slider_to_normalized(throttle_input)
            self.twist_publisher.publish(self.twist_cmd)

        # Image processing from slider values
        frame = self.bridge.imgmsg_to_cv2(data)
        height, width, channels = frame.shape

        # Setting lower constraints on camera values
        if crop_width_percent < 1:
            crop_width_percent = 1
        if rows_to_watch_percent < 1:
            rows_to_watch_percent = 1
        if rows_offset_percent < 1:
            rows_offset_percent = 1

        # Vertical crop/pan
        rows_to_watch_decimal = rows_to_watch_percent / 100
        rows_offset_decimal = rows_offset_percent / 100
        crop_width_decimal = crop_width_percent / 100
        rows_to_watch = int(height * rows_to_watch_decimal)
        rows_offset = int(height * (1 - rows_offset_decimal))

        # Horizontal crop
        start_height = int(height - rows_offset)
        bottom_height = int(start_height + rows_to_watch)
        left_width = int((width / 2) * (1 - crop_width_decimal))
        right_width = int((width / 2) * (1 + crop_width_decimal))

        img = frame[start_height:bottom_height, left_width:right_width]
        image_width = right_width-left_width
        image_height = bottom_height-start_height

        # Changing color space to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        lower_gold = np.array([self.Hue_low_gold, self.Saturation_low_gold, self.Value_low_gold])
        higher_gold = np.array([self.Hue_high_gold, self.Saturation_high_gold, self.Value_high_gold])
        
        lower_silver = np.array([self.Hue_low_silver, self.Saturation_low_silver, self.Value_low_silver])
        higher_silver = np.array([self.Hue_high_silver, self.Saturation_high_silver, self.Value_high_silver])
        
        mask_gold = cv2.inRange(hsv, lower_gold, higher_gold)
        mask_silver = cv2.inRange(hsv, lower_silver, higher_silver)

        # Inverting color filter option
        if self.Inverted_filter_gold == 1:
            bitwise_mask_gold = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_not(mask_gold))
        else:
            bitwise_mask_gold = cv2.bitwise_and(hsv, hsv, mask=mask_gold)
            
        if self.Inverted_filter_silver == 1:
            bitwise_mask_silver = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_not(mask_silver))
        else:
            bitwise_mask_silver = cv2.bitwise_and(hsv, hsv, mask=mask_silver)


        # Changing to gray color space
        gray_gold = cv2.cvtColor(bitwise_mask_gold, cv2.COLOR_BGR2GRAY)
        gray_silver = cv2.cvtColor(bitwise_mask_silver, cv2.COLOR_BGR2GRAY)

        # Changing to black and white color space
        gray_upper = 255
        (dummy_gold, blackAndWhiteImage_gold) = cv2.threshold(gray_gold, self.gray_lower_gold, gray_upper, cv2.THRESH_BINARY)
        (dummy_silver, blackAndWhiteImage_silver) = cv2.threshold(gray_silver, self.gray_lower_silver, gray_upper, cv2.THRESH_BINARY)
        # blackAndWhiteImage = cv2.adaptiveThreshold(gray, gray_upper, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
        
        # Get rid of white noise from grass
        kernel_gold = np.ones((self.kernal_size_gold, self.kernal_size_gold), np.uint8)
        kernel_silver = np.ones((self.kernal_size_silver, self.kernal_size_silver), np.uint8)
        blurred_gold = cv2.blur(blackAndWhiteImage_gold,(self.kernal_size_gold, self.kernal_size_gold))
        blurred_silver = cv2.blur(blackAndWhiteImage_silver,(self.kernal_size_silver, self.kernal_size_silver))
        erosion_gold = cv2.erode(blurred_gold, kernel_gold, iterations = self.erosion_itterations_gold)
        erosion_silver = cv2.erode(blurred_silver, kernel_silver, iterations = self.erosion_itterations_silver)
        dilation_gold = cv2.dilate(erosion_gold, kernel_gold, iterations = self.dilation_itterations_gold)
        dilation_silver = cv2.dilate(erosion_silver, kernel_silver, iterations = self.dilation_itterations_silver)
        
        
        

        # Black and white image
        (dummy_gold, blackAndWhiteImage_gold) = cv2.threshold(dilation_gold, self.gray_lower_gold, gray_upper, cv2.THRESH_BINARY)
        (dummy_silver, blackAndWhiteImage_silver) = cv2.threshold(dilation_silver, self.gray_lower_silver, gray_upper, cv2.THRESH_BINARY)

        # Finding contours in image
        contours_gold, dummy_gold = cv2.findContours(blackAndWhiteImage_gold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours_silver, dummy_silver = cv2.findContours(blackAndWhiteImage_silver, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        '''
        #My personal attampt at publishing the silver contour value 
        msg = Float32()
        msg.data = float(self.p)
        self.publisher_.publish(msg)
        string_contours = str(msg.data)
        self.get_logger().info(string_contours)
        self.p  = len(contours_silver)
       ''' 
        
        # Creating points to be drawn on image
        cam_center_line_x = int(image_width * self.camera_centerline)
        start_point = (cam_center_line_x, 0)
        end_point = (cam_center_line_x, int(bottom_height))

        start_point_thresh_pos_x = int(cam_center_line_x -  (self.error_threshold * image_width/2))
        start_point_thresh_neg_x = int(cam_center_line_x + (self.error_threshold * image_width/2))
        
        start_point_thresh_pos = (start_point_thresh_pos_x, 0)
        end_point_thresh_pos = (start_point_thresh_pos_x, int(bottom_height))

        start_point_thresh_neg = (start_point_thresh_neg_x, 0)
        end_point_thresh_neg = (start_point_thresh_neg_x, int(bottom_height))

        # Initialize lists
        cx_list = []
        cy_list = []

        # plotting contours and their centroids
        for contour in contours_gold[:self.number_of_lines]:
            [x, y], [w, h], phi = cv2.minAreaRect(contour)
            rect = cv2.minAreaRect(contour)
            if self.min_width < w < self.max_width:
                try:
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    img = cv2.drawContours(img,[box], 0, (0, 255, 0), 3)
                    m = cv2.moments(contour) # moment of contour
                    cx = int(m['m10'] / m['m00']) # x_pos of centroid
                    cy = int(m['m01'] / m['m00']) # y_pos of centroid
                    cx_list.append(cx)
                    cy_list.append(cy)
                    cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
                    img = cv2.line(img, start_point, end_point, (0,255,0), 4)
                    img = cv2.line(img, start_point_thresh_pos, end_point_thresh_pos, (0,0,255), 2)
                    img = cv2.line(img, start_point_thresh_neg, end_point_thresh_neg, (0,0,255), 2)
                except ZeroDivisionError:
                    pass
        try:
            # When more than 1 road mark is found
            if len(cx_list) > 1:
                error_list = []
                count = 0

                # calculate normalized errors for all detected road lines
                for cx_pos in cx_list:
                    error = float((cx_pos - cam_center_line_x) / cam_center_line_x)
                    error_list.append(error)

                # finding average normalized error of all road lines
                avg_error = (sum(error_list) / float(len(error_list)))

                # check difference in normalized error from closest to furthest road line
                p_horizon_diff = abs(error_list[0] - error_list[-1]) 
                
                # if path is approximately straight, then steer towards average normalized error
                if abs(p_horizon_diff) <= self.error_threshold:
                    self.ek = avg_error
                    pixel_error = int(cam_center_line_x * (1 + self.ek))
                    mid_x, mid_y = pixel_error, int((image_height / 2))
                    self.get_logger().info(f"Straight curve: [tracking error: {self.ek}], [tracking angle: {phi}]")
                
                # if path is curved, then steer towards minimum normalized error
                else: 
                    # exclude any road lines within error threshold by making their error large
                    for error in error_list:
                        if abs(error) < self.error_threshold:
                            error = 1
                            error_list[count] = error
                        count+=1
                    
                    # min error (closest roadline)
                    self.ek = min(error_list, key=abs)

                    # get index of min error for plotting
                    error_x_index = error_list.index(min(error_list, key=abs))
                    mid_x, mid_y = cx_list[error_x_index], cy_list[error_x_index]
                    self.get_logger().info(f"Curvy road: [tracking error: {self.ek}], [tracking angle: {phi}]")

                # ploting lines and circles of contours 
                cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
                start_point_error = (cam_center_line_x, mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)

            # When only 1 road mark was found    
            elif len(cx_list) == 1:
                mid_x, mid_y = cx_list[0], cy_list[0]
                start_point_error = (cam_center_line_x, mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)
                cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)

                self.ek = float((mid_x - cam_center_line_x) / cam_center_line_x)
                self.get_logger().info(f"Only detected one line: [tracking error: {self.ek}], [tracking angle: {phi}]")

            # When Nothing was found
            else:
                self.get_logger().info(f"Nothing detected")
            
            # clean slate
            cx_list = []
            cy_list = []
        except ValueError:
            pass

        # plotting results
        cv2.imshow(IMG_WINDOW_NAME, img)
        cv2.imshow(MASK_WINDOW_NAME_1, mask_gold)
        cv2.imshow(MASK_WINDOW_NAME_2, mask_silver)
        cv2.imshow(BW_WINDOW_NAME_1, blackAndWhiteImage_gold)
        cv2.imshow(BW_WINDOW_NAME_2, blackAndWhiteImage_silver)
        cv2.waitKey(1)

        # Write files to yaml file for storage. TODO write individual yaml files for each node
        color_config_path = str('/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/config/ros_racer_calibration.yaml')
        f = open(color_config_path, "w")
        f.write(
            f"{LD_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            
            f"    Hue_low_gold : {self.Hue_low_gold} \n"
            f"    Hue_high_gold : {self.Hue_high_gold} \n"
            f"    Saturation_low_gold : {self.Saturation_low_gold} \n"
            f"    Saturation_high_gold : {self.Saturation_high_gold} \n"
            f"    Value_low_gold : {self.Value_low_gold} \n"
            f"    Value_high_gold : {self.Value_high_gold} \n"
            
            f"    Hue_low_silver : {self.Hue_low_silver} \n"
            f"    Hue_high_silver : {self.Hue_high_silver} \n"
            f"    Saturation_low_silver : {self.Saturation_low_silver} \n"
            f"    Saturation_high_silver : {self.Saturation_high_silver} \n"
            f"    Value_low_silver : {self.Value_low_silver} \n"
            f"    Value_high_silver : {self.Value_high_silver} \n"
            
            f"    number_of_lines : {self.number_of_lines} \n"
            f"    error_threshold : {self.error_threshold} \n"
            f"    Width_min : {self.min_width} \n"
            f"    Width_max : {self.max_width} \n"
            
            f"    gray_lower_gold : {self.gray_lower_gold} \n"
            f"    Inverted_filter_gold : {self.Inverted_filter_gold} \n"
            f"    kernal_size_gold : {self.kernal_size_gold} \n"
            f"    erosion_itterations_gold : {self.erosion_itterations_gold} \n"
            f"    dilation_itterations_gold : {self.dilation_itterations_gold} \n"
            
            f"    gray_lower_silver : {self.gray_lower_silver} \n"
            f"    Inverted_filter_silver : {self.Inverted_filter_silver} \n"
            f"    kernal_size_silver : {self.kernal_size_silver} \n"
            f"    erosion_itterations_silver : {self.erosion_itterations_silver} \n"
            f"    dilation_itterations_silver : {self.dilation_itterations_silver} \n"
            
            f"    camera_start_height : {start_height} \n"
            f"    camera_bottom_height : {bottom_height} \n"
            f"    camera_left_width : {left_width} \n"
            f"    camera_right_width : {right_width} \n"
            f"    camera_centerline : {self.camera_centerline} \n"
            f"{CALIBRATION_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            
            f"    Hue_low_gold : {self.Hue_low_gold} \n"
            f"    Hue_high_gold : {self.Hue_high_gold} \n"
            f"    Saturation_low_gold : {self.Saturation_low_gold} \n"
            f"    Saturation_high_gold : {self.Saturation_high_gold} \n"
            f"    Value_low_gold : {self.Value_low_gold} \n"
            f"    Value_high_gold : {self.Value_high_gold} \n"
            
            f"    Hue_low_silver : {self.Hue_low_silver} \n"
            f"    Hue_high_silver : {self.Hue_high_silver} \n"
            f"    Saturation_low_silver : {self.Saturation_low_silver} \n"
            f"    Saturation_high_silver : {self.Saturation_high_silver} \n"
            f"    Value_low_silver : {self.Value_low_silver} \n"
            f"    Value_high_silver : {self.Value_high_silver} \n"
            
            f"    number_of_lines : {self.number_of_lines} \n"
            f"    error_threshold : {self.error_threshold} \n"
            f"    Width_min : {self.min_width} \n"
            f"    Width_max : {self.max_width} \n"
            
            f"    gray_lower_gold : {self.gray_lower_gold} \n"
            f"    Inverted_filter_gold : {self.Inverted_filter_gold} \n"
            f"    kernal_size_gold : {self.kernal_size_gold} \n"
            f"    erosion_itterations_gold : {self.erosion_itterations_gold} \n"
            f"    dilation_itterations_gold : {self.dilation_itterations_gold} \n"
            
            f"    gray_lower_silver : {self.gray_lower_silver} \n"
            f"    Inverted_filter_silver : {self.Inverted_filter_silver} \n"
            f"    kernal_size_silver : {self.kernal_size_silver} \n"
            f"    erosion_itterations_silver : {self.erosion_itterations_silver} \n"
            f"    dilation_itterations_silver : {self.dilation_itterations_silver} \n"
            
            f"    camera_centerline : {self.camera_centerline} \n"
            f"{LG_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    Kp_steering : {self.Kp_steering} \n"
            f"    Ki_steering : {self.Ki_steering} \n"
            f"    Kd_steering : {self.Kd_steering} \n"
            f"    zero_throttle  : {self.zero_throttle } \n"
            f"    max_throttle : {self.max_throttle} \n"
            f"    min_throttle : {self.min_throttle} \n"
            f"    error_threshold : {self.error_threshold} \n"
            f"    max_right_steering : {self.max_right_steering} \n"
            f"    max_left_steering : {self.max_left_steering} \n"
            f"{VESC_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    max_rpm : {self.max_rpm} \n"
            f"    steering_polarity : {self.steering_polarity} \n"
            f"    throttle_polarity : {self.throttle_polarity} \n"
            f"    zero_throttle  : {self.zero_throttle } \n"
            f"    max_throttle : {self.max_throttle} \n"
            f"    min_throttle : {self.min_throttle} \n"
            f"    max_right_steering : {self.max_right_steering} \n"
            f"    straight_steering : {self.straight_steering} \n"
            f"    max_left_steering : {self.max_left_steering} \n"
            f"{ADA_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    steering_polarity : {self.steering_polarity} \n"
            f"    throttle_polarity : {self.throttle_polarity} \n"
            f"    zero_throttle  : {self.zero_throttle } \n"
            f"    max_throttle : {self.max_throttle} \n"
            f"    min_throttle : {self.min_throttle} \n"
            f"    max_right_steering : {self.max_right_steering} \n"
            f"    straight_steering : {self.straight_steering} \n"
            f"    max_left_steering : {self.max_left_steering} \n"
        )
        f.close()

    def clamp(self, data, upper_bound, lower_bound=None):
            if lower_bound==None:
                lower_bound = -upper_bound # making lower bound symmetric about zero
            if data < lower_bound:
                data_c = lower_bound
            elif data > upper_bound:
                data_c = upper_bound
            else:
                data_c = data
            return data_c 
            

def main(args=None):
    rclpy.init(args=args)
    CAL_publisher = Calibration()
    try:
        rclpy.spin(CAL_publisher)
        CAL_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print(f"\nShutting down {CALIBRATION_NODE_NAME}...")

        # Stop the car
        CAL_publisher.twist_cmd.linear.x = CAL_publisher.zero_throttle
        CAL_publisher.twist_publisher.publish(CAL_publisher.twist_cmd)

        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        CAL_publisher.destroy_node()
        rclpy.shutdown()
        print(f"{CALIBRATION_NODE_NAME} shut down successfully.")
        
        



if __name__ == '__main__':
    main()

