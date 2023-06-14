# Imports
import cv2
import depthai as dai
import contextlib
import os
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

import sys
from pathlib import Path
from std_msgs.msg import Float32, Int32, Int32MultiArray, String
from geometry_msgs.msg import Twist
import time
import os

NODE_NAME = 'shastaCameraTesting'

class MultiCamNode(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        self.device_info = dai.Device.getAllAvailableDevices()
        self.q_rgb_list = []
        self.num_devices = len(self.device_info)
        self.cam_publishers = []
        for i in range(self.num_devices):
             self.cam_publishers.append(self.create_publisher(Image, "camera/color/image_raw_" + str(i), 10))
        self.bridge = CvBridge()
        
    def getPipeline(self, preview_res = (1448, 568)):
        # # Start defining a pipeline
        # pipeline = dai.Pipeline()

        # # Define a source - color camera
        # cam_rgb = pipeline.create(dai.node.ColorCamera)
        # # For the demo, just set a larger RGB preview size for OAK-D
        # cam_rgb.setPreviewSize(preview_res[0], preview_res[1]) # FIX ME, need to match what ever pipeline we are using
        # cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        # cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        # cam_rgb.setInterleaved(False)

        # # Create output
        # xout_rgb = pipeline.create(dai.node.XLinkOut)
        # xout_rgb.setStreamName("rgb")
        # cam_rgb.preview.link(xout_rgb.input)

        # return pipeline
        
        #Get DepthAI Blob File Pathways
        nnPath = str((Path(__file__) / Path('~/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/ucsd_robocar_lane_detection2_pkg/depthai-python/examples/models/mobilenet-ssd_openvino_2021.4_6shave.blob')).resolve().absolute())
        if len(sys.argv) > 1:
            nnPath = sys.argv[1]
        if not Path(nnPath).exists():
            raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

        # Create pipeline
        pipeline = dai.Pipeline()

        # Define source and outputs
        camRgb = pipeline.create(dai.node.ColorCamera)
        camRgb.setInterleaved(False)
        camControlIn = pipeline.create(dai.node.XLinkIn)
        camControlIn.setStreamName('camControl')
        camControlIn.out.link(camRgb.inputControl)
        camRgb.setPreviewSize(preview_res[0], preview_res[1]) # FIX ME, need to match what ever pipeline we are using
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        # Define a neural network that will make predictions based on the source frames
        nn = pipeline.create(dai.node.MobileNetDetectionNetwork)
        nn.setConfidenceThreshold(0.5)
        nn.setBlobPath(nnPath)
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)
        camRgb.preview.link(nn.input)

        # Linking
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        camRgb.preview.link(xoutRgb.input)
        nnOut = pipeline.create(dai.node.XLinkOut)
        nnOut.setStreamName("nn")
        nn.out.link(nnOut.input)

        # MobilenetSSD label texts
        labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
                    "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

        return pipeline

    def camera_initialization(self, debug = False, path = "./"):

        # https://docs.python.org/3/library/contextlib.html#contextlib.ExitStack
        with contextlib.ExitStack() as stack:
            device_infos = dai.Device.getAllAvailableDevices()
            if len(device_infos) == 0:
                raise RuntimeError("No devices found!")
            else:
                print("Found", len(device_infos), "devices")

            for device_info in device_infos:
                openvino_version = dai.OpenVINO.Version.VERSION_2021_4
                usb2_mode = False
                device = stack.enter_context(dai.Device(openvino_version, device_info, usb2_mode))

                # Note: currently on POE, DeviceInfo.getMxId() and Device.getMxId() are different!
                print("=== Connected to " + device_info.getMxId())
                mxid = device.getMxId()
                cameras = device.getConnectedCameras()
                usb_speed = device.getUsbSpeed()
                print("   >>> MXID:", mxid)
                print("   >>> Cameras:", *[c.name for c in cameras])
                print("   >>> USB speed:", usb_speed.name)


                # Get a customized pipeline based on identified device type
                pipeline = self.getPipeline()
                print("   >>> Loading pipeline for: OAK-D-LITE")
                device.startPipeline(pipeline)

                # Output queue will be used to get the rgb frames from the output defined above
                # q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                # stream_name = "rgb-" + mxid + "-" + "OAK-D"
                # self.q_rgb_list.append((q_rgb, stream_name))
                
                # Output queues will be used to get the rgb frames and nn data from the outputs defined above
                qControl = device.getInputQueue(name="camControl")
                qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
                detections = []

            if debug:
                    self.image_display_opencv(path = path)
                
            else:
                while True:
                    for i, (q_rgb, _) in enumerate(self.q_rgb_list):
                        in_rgb = q_rgb.tryGet()
                        if in_rgb is not None:
                            img_msg = self.bridge.cv2_to_imgmsg(in_rgb.getCvFrame(), "bgr8")
                            self.cam_publishers[i].publish(img_msg)
                        inDet = qDet.tryGet()
                        if inDet is not None:
                            detections = inDet.detections
                            for d in detections:
                                if(labelMap[d.label] == "person"): 
                                    print(1)
                                else: 
                                    print(0)

                    if cv2.waitKey(1) == ord('q'):
                        break

    
    def image_display_opencv(self, path): # debug purpose only
        img_cnt = 0
        while True:
            for q_rgb, stream_name in self.q_rgb_list:
                in_rgb = q_rgb.tryGet()
                if in_rgb is not None:
                    cv2.imshow(stream_name, in_rgb.getCvFrame())
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('s'):
                        cv2.imwrite(os.path.join(path, str(img_cnt) + '.bmp'), in_rgb.getCvFrame())
                        print("Saved image: ", img_cnt)
                        img_cnt += 1
                    elif key == ord('q'):
                        exit("user quit")

def main():

    rclpy.init()
    cam_node = MultiCamNode()
    try:
    	cam_node.camera_initialization(debug=False, path ='/home/projects/sensor2_ws/src/camera/oakd_debug/cv_img_save')
    except KeyboardInterrupt:
        print(f"\nShutting down {NODE_NAME}...")
        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        cam_node.destroy_node()
        rclpy.shutdown()
        print(f"{NODE_NAME} shut down successfully.")


if __name__ == "main":
    main()




# class PersonPublisher(Node):
#     def __init__(self):
#         super().__init__(CAMERA_NODE_NAME)
#         self.publisher_ = self.create_publisher(Int32, TOPIC_NODE_NAME , 10)
#         timer_period = 1/30  # seconds
#         self.timer = self.create_timer(timer_period, self.find_person)
#         self.i = 0

#     def find_person(self):    
#         # Default detection var.    
#         person_detected = 0
        
#         # Connect to device and start pipeline
#         with dai.Device(pipeline) as device:
#             # Output queues will be used to get the rgb frames and nn data from the outputs defined above
#             qControl = device.getInputQueue(name="camControl")
#             qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
#             qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
#             detections = []
#             while True:
#                 inDet = qDet.tryGet()
#                 if inDet is not None:
#                     detections = inDet.detections
#                     for d in detections:
#                         if(labelMap[d.label] == "person"): 
#                             person_detected = 1
#                             #print(1)
#                         else: 
#                             person_detected = 0
#                             #print(0)
        
#         self.person_detected = person_detected
#         self.publisher_.publish(person_detected)
#         self.get_logger().info(f'\nPerson Detected: {self.person_detected}')
#         self.i = self.person_detected 

# def main(args=None):
#     rclpy.init(args=args)
#     person_publisher = PersonPublisher()
#     try:
#         rclpy.spin(person_publisher)
#         person_publisher.destroy_node()
#         rclpy.shutdown()
#     except KeyboardInterrupt:
#         print(f"\nShutting down {CAMERA_NODE_NAME}...")
#         # Kill cv2 windows and node
#         person_publisher.destroy_node()
#         rclpy.shutdown()
#         print(f"{CAMERA_NODE_NAME} shut down successfully.")


# if __name__ == '__main__':
#     main()
