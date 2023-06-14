# Oakd Node File from sensor2_pkg

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

NODE_NAME = "oakd_node"


class MultiCamNode(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        self.device_info = dai.Device.getAllAvailableDevices()
        self.q_rgb_list = []
        self.num_devices = len(self.device_info)
        self.cam_publishers = []

        # My attempt at publishing Int of person detected
        self.labelMap = []
        self.person_detected = self.create_publisher(Int32, 'person_detected', 10)
        self.i = 0


        for i in range(self.num_devices):
             self.cam_publishers.append(self.create_publisher(Image, 'camera/color/image_raw', 10))
        self.bridge = CvBridge()
        

    #Function for getting pipeline
    def getPipeline(self, preview_res = (300, 300)):

        #Create Path for Blob
        nnPath = str((Path('src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/ucsd_robocar_lane_detection2_pkg/depthai-python/examples/models/mobilenet-ssd_openvino_2021.4_6shave.blob')).resolve().absolute())
        
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
        self.labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
                    "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

        return pipeline

    def camera_initialization(self):

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


                qControl = device.getInputQueue(name="camControl")
                qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                stream_name = "rgb-" + mxid + "-" + "OAK-D"
                self.q_rgb_list.append((qRgb, stream_name))
                qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
                detections = []

                while True:
                    for i, (qRgb, _) in enumerate(self.q_rgb_list):
                        in_rgb = qRgb.tryGet()
                        if in_rgb is not None:
                            img_msg = self.bridge.cv2_to_imgmsg(in_rgb.getCvFrame(), "bgr8")
                            self.cam_publishers[i].publish(img_msg)
                        
                        # My attempt at detecting people, publishing 1 if true and 0 if false
                        inDet = qDet.tryGet()
                        if inDet is not None:
                            detections = inDet.detections
                            for d in detections:
                                if(self.labelMap[d.label] == "person"):
                                    person = 1

                                else:
                                    person = 0
                                msg = Int32()
                                msg.data = person
                                self.person_detected.publish(msg)
                                #self.get_logger().info(f'\n person detected: {msg.data}')
                                self.i += 1
                    if cv2.waitKey(1) == ord('q'):
                        break


def main():
    rclpy.init()
    cam_node = MultiCamNode()
    try:
    	cam_node.camera_initialization()
    except KeyboardInterrupt:
        print(f"\nShutting down {NODE_NAME}...")
        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        cam_node.destroy_node()
        rclpy.shutdown()
        print(f"{NODE_NAME} shut down successfully.")


if __name__ == "main":
    main()
