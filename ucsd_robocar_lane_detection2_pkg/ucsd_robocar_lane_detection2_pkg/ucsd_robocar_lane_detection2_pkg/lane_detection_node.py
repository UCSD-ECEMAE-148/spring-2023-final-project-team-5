# Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Int32MultiArray, String
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

# Nodes in this program
NODE_NAME = 'lane_detection_node'
LG_NODE_NAME = 'lane_guidance_node'

# Topics subcribed/published to in this program
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
CENTROID_TOPIC_NAME = '/centroid'


class LaneDetection(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.centroid_error_publisher = self.create_publisher(Float32, CENTROID_TOPIC_NAME, 10)
        self.centroid_error_publisher
        self.centroid_error = Float32()
        self.camera_subscriber = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.locate_centroid, 10)
        self.camera_subscriber
        
        self.obstacle_value = 0.0
        
        #My personal attempt at publishing a value to lane guidance
        self.publisher_ = self.create_publisher(Float32, NODE_NAME, 10)
        self.p = 0.0
        
         #My personal attempt at subscribing to data from the calibration node 
        self.subscription = self.create_subscription(
            String,
            LG_NODE_NAME,
            self.anotherCallFunction,
            10)
        self.subscription  # prevent unused variable warning
        
        self.bridge = CvBridge()
        self.max_num_lines_detected = 10
        self.image_width = 0
        self.error_threshold = 0.1
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Hue_low_gold', 1),
                ('Hue_high_gold', 1),
                ('Saturation_low_gold', 1),
                ('Saturation_high_gold', 1),
                ('Value_low_gold', 1),
                ('Value_high_gold', 1),
                ('gray_lower_gold', 1),
                ('inverted_filter_gold', 0),
                ('kernal_size_gold',1),
                ('erosion_itterations_gold',1),
                ('dilation_itterations_gold',1),
                
                ('Hue_low_silver', 1),
                ('Hue_high_silver', 1),
                ('Saturation_low_silver', 1),
                ('Saturation_high_silver', 1),
                ('Value_low_silver', 1),
                ('Value_high_silver', 1),
                ('gray_lower_silver', 1),
                ('inverted_filter_silver', 0),
                ('kernal_size_silver',1),
                ('erosion_itterations_silver',1),
                ('dilation_itterations_silver',1),
                
                ('number_of_lines', 0),
                ('error_threshold', 0),
                ('Width_min', 1),
                ('Width_max', 1),
                ('camera_start_height', 1),
                ('camera_bottom_height', 1),
                ('camera_left_width', 1),
                ('camera_right_width', 1),
                ('camera_centerline',0.5),
                ('debug_cv', 0)
            ])
        self.Hue_low_gold = self.get_parameter('Hue_low_gold').value
        self.Hue_high_gold = self.get_parameter('Hue_high_gold').value
        self.Saturation_low_gold = self.get_parameter('Saturation_low_gold').value
        self.Saturation_high_gold = self.get_parameter('Saturation_high_gold').value
        self.Value_low_gold = self.get_parameter('Value_low_gold').value
        self.Value_high_gold = self.get_parameter('Value_high_gold').value
        self.gray_lower_gold = self.get_parameter('gray_lower_gold').value
        self.inverted_filter_gold = self.get_parameter('inverted_filter_gold').value
        self.kernal_size_gold = self.get_parameter('kernal_size_gold').value
        self.erosion_itterations_gold = self.get_parameter('erosion_itterations_gold').value
        self.dilation_itterations_gold = self.get_parameter('dilation_itterations_gold').value
        
        self.Hue_low_silver = self.get_parameter('Hue_low_silver').value
        self.Hue_high_silver = self.get_parameter('Hue_high_silver').value
        self.Saturation_low_silver = self.get_parameter('Saturation_low_silver').value
        self.Saturation_high_silver = self.get_parameter('Saturation_high_silver').value
        self.Value_low_silver = self.get_parameter('Value_low_silver').value
        self.Value_high_silver = self.get_parameter('Value_high_silver').value
        self.gray_lower_silver = self.get_parameter('gray_lower_silver').value
        self.inverted_filter_silver = self.get_parameter('inverted_filter_silver').value
        self.kernal_size_silver = self.get_parameter('kernal_size_silver').value
        self.erosion_itterations_silver = self.get_parameter('erosion_itterations_silver').value
        self.dilation_itterations_silver = self.get_parameter('dilation_itterations_silver').value
        
        self.number_of_lines = self.get_parameter('number_of_lines').value
        self.error_threshold = self.get_parameter('error_threshold').value
        self.min_width = self.get_parameter('Width_min').value
        self.max_width = self.get_parameter('Width_max').value
        self.start_height = self.get_parameter('camera_start_height').value
        self.bottom_height = self.get_parameter('camera_bottom_height').value
        self.left_width = self.get_parameter('camera_left_width').value
        self.right_width = self.get_parameter('camera_right_width').value
        self.camera_centerline = self.get_parameter('camera_centerline').value
        self.debug_cv = self.get_parameter('debug_cv').value
        self.get_logger().info(
            f'\nHue_low_gold: {self.Hue_low_gold}'
            f'\nHue_high_gold: {self.Hue_high_gold}'
            f'\nSaturation_low_gold: {self.Saturation_low_gold}'
            f'\nSaturation_high_gold: {self.Saturation_high_gold}'
            f'\nValue_low_gold: {self.Value_low_gold}'
            f'\nValue_high_gold: {self.Value_high_gold}'
            f'\ngray_lower_gold: {self.gray_lower_gold}'
            f'\ninverted_filter_gold: {self.inverted_filter_gold}'
            f'\nkernal_size_gold: {self.kernal_size_gold}'
            f'\nerosion_itterations_gold: {self.erosion_itterations_gold}'
            f'\ndilation_itterations_gold: {self.dilation_itterations_gold}'
            
            f'\nHue_low_silver: {self.Hue_low_silver}'
            f'\nHue_high_silver: {self.Hue_high_silver}'
            f'\nSaturation_low_silver: {self.Saturation_low_silver}'
            f'\nSaturation_high_silver: {self.Saturation_high_silver}'
            f'\nValue_low_silver: {self.Value_low_silver}'
            f'\nValue_high_silver: {self.Value_high_silver}'
            f'\ngray_lower_silver: {self.gray_lower_silver}'
            f'\ninverted_filter_silver: {self.inverted_filter_silver}'
            f'\nkernal_size_silver: {self.kernal_size_silver}'
            f'\nerosion_itterations_silver: {self.erosion_itterations_silver}'
            f'\ndilation_itterations_silver: {self.dilation_itterations_silver}'
            
            f'\nnumber_of_lines: {self.number_of_lines}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nmin_width: {self.min_width}'
            f'\nmax_width: {self.max_width}'
            f'\nstart_height: {self.start_height}'
            f'\nbottom_height: {self.bottom_height}'
            f'\nleft_width: {self.left_width}'
            f'\nright_width: {self.right_width}'
            f'\ncamera_centerline: {self.camera_centerline}'
            f'\ndebug_cv: {self.debug_cv}')
            
    def anotherCallFunction(self, obst):
       self.obstacle_value = float(obst.data)
        
    def locate_centroid(self, data):
       # self.get_logger().info('Obstacle Detection Ranges: "%s"' % str(self.obstacle_value))
        # Image processing from rosparams
        frame = self.bridge.imgmsg_to_cv2(data)

        self.image_width = int(self.right_width - self.left_width)
        self.image_width = int(self.right_width - self.left_width)

        img = frame[self.start_height:self.bottom_height, self.left_width:self.right_width]

        image_height = self.bottom_height-self.start_height

        # changing color space to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_gold = np.array([self.Hue_low_gold, self.Saturation_low_gold, self.Value_low_gold])
        upper_gold = np.array([self.Hue_high_gold, self.Saturation_high_gold, self.Value_high_gold])
        mask_gold = cv2.inRange(hsv, lower_gold, upper_gold)
        
        lower_silver = np.array([self.Hue_low_silver, self.Saturation_low_silver, self.Value_low_silver])
        upper_silver = np.array([self.Hue_high_silver, self.Saturation_high_silver, self.Value_high_silver])
        mask_silver = cv2.inRange(hsv, lower_silver, upper_silver)

        if self.inverted_filter_gold == 1:
            bitwise_mask_gold = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_not(mask_gold))
        else:
            bitwise_mask_gold = cv2.bitwise_and(hsv, hsv, mask=mask_gold)
            
            
        if self.inverted_filter_silver == 1:
            bitwise_mask_silver = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_not(mask_silver))
        else:
            bitwise_mask_silver = cv2.bitwise_and(hsv, hsv, mask=mask_silver)


        # changing to gray color space
        gray_gold = cv2.cvtColor(bitwise_mask_gold, cv2.COLOR_BGR2GRAY)
        gray_silver = cv2.cvtColor(bitwise_mask_silver, cv2.COLOR_BGR2GRAY)

        # changing to black and white color space
        gray_upper = 255
        (dummy_gold, blackAndWhiteImage_gold) = cv2.threshold(gray_gold, self.gray_lower_gold, gray_upper, cv2.THRESH_BINARY)
        (dummy_silver, blackAndWhiteImage_silver) = cv2.threshold(gray_silver, self.gray_lower_silver, gray_upper, cv2.THRESH_BINARY)
        
        # get rid of white noise from grass
        kernel_gold = np.ones((self.kernal_size_gold, self.kernal_size_gold), np.uint8)
        blurred_gold = cv2.blur(blackAndWhiteImage_gold,(self.kernal_size_gold, self.kernal_size_gold))
        erosion_gold = cv2.erode(blurred_gold, kernel_gold, iterations = self.erosion_itterations_gold)
        dilation_gold = cv2.dilate(erosion_gold, kernel_gold, iterations = self.dilation_itterations_gold)
        
        kernel_silver = np.ones((self.kernal_size_silver, self.kernal_size_silver), np.uint8)
        blurred_silver = cv2.blur(blackAndWhiteImage_silver,(self.kernal_size_silver, self.kernal_size_silver))
        erosion_silver = cv2.erode(blurred_silver, kernel_silver, iterations = self.erosion_itterations_silver)
        dilation_silver = cv2.dilate(erosion_silver, kernel_silver, iterations = self.dilation_itterations_silver)

        (dummy_gold, blackAndWhiteImage_gold) = cv2.threshold(dilation_gold, self.gray_lower_gold, gray_upper, cv2.THRESH_BINARY)
        contours_gold, dummy_gold = cv2.findContours(blackAndWhiteImage_gold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        (dummy_silver, blackAndWhiteImage_silver) = cv2.threshold(dilation_silver, self.gray_lower_silver, gray_upper, cv2.THRESH_BINARY)
        contours_silver, dummy_silver = cv2.findContours(blackAndWhiteImage_silver, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        #My personal attampt at publishing the silver contour value 
        msg = Float32()
        msg.data = float(self.p)
        self.publisher_.publish(msg)
        string_contours = str(msg.data)
        #self.get_logger().info(string_contours)
        self.p  = len(contours_silver)
        
        # Defining points of a line to be drawn for visualizing error
        cam_center_line_x = int(self.image_width * self.camera_centerline)
        start_point = (cam_center_line_x,0)
        end_point = (cam_center_line_x, int(self.bottom_height))

        start_point_thresh_pos_x = int(cam_center_line_x -  (self.error_threshold * self.image_width/2))
        start_point_thresh_neg_x = int(cam_center_line_x + (self.error_threshold * self.image_width/2))

        start_point_thresh_pos = (start_point_thresh_pos_x, 0)
        end_point_thresh_pos = (start_point_thresh_pos_x, int(self.bottom_height))

        start_point_thresh_neg = (start_point_thresh_neg_x, 0)
        end_point_thresh_neg = (start_point_thresh_neg_x, int(self.bottom_height))

        # Setting up data arrays
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
                    m = cv2.moments(contour)
                    cx = int(m['m10'] / m['m00'])
                    cy = int(m['m01'] / m['m00'])
                    cx_list.append(cx)
                    cy_list.append(cy)
                    cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
                    img = cv2.line(img, start_point, end_point, (0,255,0), 4)
                    img = cv2.line(img, start_point_thresh_pos, end_point_thresh_pos, (0,0,255), 2)
                    img = cv2.line(img, start_point_thresh_neg, end_point_thresh_neg, (0,0,255), 2)
                except ZeroDivisionError:
                    pass
        # Further image processing to determine optimal steering value
        try:
            # When more than 1 road mark is found
            if len(cx_list) > 1:
                error_list = []
                count = 0

                # calculate errors for all detected road lines
                for cx_pos in cx_list:
                    error = float((cx_pos - cam_center_line_x) / cam_center_line_x)
                    error_list.append(error)

                # finding average error of all road lines
                avg_error = (sum(error_list) / float(len(error_list)))

                # check difference in error from closest to furthest road line
                p_horizon_diff = abs(error_list[0] - error_list[-1])

                # if path is approximately straight, then steer towards average error
                if abs(p_horizon_diff) <= self.error_threshold:
                    error_x = avg_error
                    pixel_error = int(cam_center_line_x * (1 + error_x))
                    mid_x, mid_y = pixel_error, int((image_height/2))
                    #self.get_logger().info(f"Straight curve: [tracking error: {error_x}], [tracking angle: {phi}]")

                # if path is curved, then steer towards minimum error
                else:
                    # exclude any road lines within error threshold by making their error large
                    for error in error_list:
                        if abs(error) < self.error_threshold:
                            error = 1
                            error_list[count] = error
                        count+=1

                    # getting min error (closest roadline)
                    error_x = min(error_list, key=abs)

                    # get index of min error for plotting
                    error_x_index = error_list.index(min(error_list, key=abs))
                    mid_x, mid_y = cx_list[error_x_index], cy_list[error_x_index]
                    #self.get_logger().info(f"Curvy road: [tracking error: {error_x}], [tracking angle: {phi}]")

                # plotting roadline to be tracked
                cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
                start_point_error = (cam_center_line_x, mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)

                # publish error data
                self.centroid_error.data = float(error_x)
                self.centroid_error_publisher.publish(self.centroid_error)

            # When only 1 road mark was found
            elif len(cx_list) == 1:
                mid_x, mid_y = cx_list[0], cy_list[0]
                error_x = float((mid_x - cam_center_line_x) / cam_center_line_x)
                start_point_error = (cam_center_line_x, mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)
                cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)

                self.centroid_error.data = error_x
                self.centroid_error_publisher.publish(self.centroid_error)
                #self.get_logger().info(f"Only detected one line: [tracking error: {error_x}], [tracking angle: {phi}]")

            # When Nothing was found
            else:
                self.get_logger().info(f"Nothing detected")

            # clean slate
            error_list = [0] * self.number_of_lines
            cx_list = []
            cy_list = []
        except ValueError:
            pass

        # plotting results
        self.debug_cv = self.get_parameter('debug_cv').value # ability to update debug in real-time
        if self.debug_cv:
            cv2.imshow('img', img)
            cv2.imshow('blackAndWhiteImage_gold', blackAndWhiteImage_gold)
            cv2.imshow('blackAndWhiteImage_silver', blackAndWhiteImage_silver)
            cv2.waitKey(1)
        else:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    centroid_publisher = LaneDetection()
    try:
        rclpy.spin(centroid_publisher)
        centroid_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        centroid_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')

        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        centroid_publisher.destroy_node()
        rclpy.shutdown()
        centroid_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()