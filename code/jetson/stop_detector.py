#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 StopSign Detection Node (Headless). Publishes stop/no-stop signals to /stop_cmd_twist.
Removed saving JPG images. Assumes model files (.blob and .json) are in the same directory as this script.
"""

import os
import json
import time

import depthai as dai
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LocalOAK:
    """Local implementation using official YOLOv8 decoding to detect StopSigns"""

    def __init__(self, blob_path, config_path, confidence=0.9, overlap=0.5, rgb=True, depth=True):
        self.blob_path = blob_path
        self.config_path = config_path
        self.confidence = confidence
        self.overlap = overlap
        self.rgb = rgb
        self.depth = depth
        self.bridge = CvBridge()
        self.image_pub = None  


        # Check that model files exist
        if not os.path.exists(blob_path):
            raise FileNotFoundError(f"Blob file not found: {blob_path}")
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        # Load JSON configuration
        self._load_config()

        # Set up DepthAI pipeline
        self._setup_pipeline()

        # Connect to device
        self.device = dai.Device(self.pipeline)

        # Retrieve output queues
        self.q_rgb = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
        self.q_depth = self.device.getOutputQueue("depth", maxSize=4, blocking=False)
        self.q_det = self.device.getOutputQueue("detections", maxSize=4, blocking=False)

        print(f"✅ LocalOAK initialized with:")
        print(f"   Blob:   {blob_path}")
        print(f"   Config: {config_path}")
        print(f"   Classes: {self.num_classes}")
        print(f"   Input size: {self.input_size}x{self.input_size}")

    def _load_config(self):
        """Load YOLOv8 JSON configuration file"""
        with open(self.config_path, 'r') as f:
            self.config = json.load(f)

        nn_config = self.config['nn_config']['NN_specific_metadata']
        self.num_classes = nn_config['classes']
        self.coordinates = nn_config['coordinates']
        self.anchors = nn_config['anchors']
        self.anchor_masks = nn_config['anchor_masks']
        self.iou_threshold = nn_config['iou_threshold']
        self.confidence_threshold = nn_config['confidence_threshold']
        self.labels = self.config['mappings']['labels']

        # Parse input size string, e.g. "416x416" -> 416
        input_size_str = self.config['nn_config']['input_size']
        self.input_size = int(input_size_str.split('x')[0])

        # Find the class index for "stop sign"
        self.stop_sign_class_id = None
        for i, label in enumerate(self.labels):
            if 'stop sign' in label.lower():
                self.stop_sign_class_id = i
                break

        print(f"📋 Configuration loaded:")
        print(f"   Classes: {self.num_classes}")
        print(f"   Input size: {self.input_size}x{self.input_size}")
        print(f"   Stop sign class ID: {self.stop_sign_class_id}")
        print(f"   Confidence threshold: {self.confidence_threshold}")
        print(f"   IoU threshold: {self.iou_threshold}")

    def _setup_pipeline(self):
        """Set up DepthAI pipeline with YOLOv8 network"""
        self.pipeline = dai.Pipeline()

        # RGB camera
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(self.input_size, self.input_size)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(30)

        # Depth cameras
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        depth = self.pipeline.create(dai.node.StereoDepth)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)

        # Try new API, otherwise fallback to older calls
        try:
            mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
            mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
            print("✅ Using new API: setBoardSocket")
        except AttributeError:
            try:
                mono_left.setCamera("left")
                mono_right.setCamera("right")
                print("✅ Using old API: setCamera")
            except AttributeError:
                try:
                    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
                    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
                    print("✅ Using new API: setBoardSocket (CAM_B/CAM_C)")
                except Exception as e:
                    print(f"❌ Unable to set mono camera sockets: {e}")
                    raise

        depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        depth.setLeftRightCheck(True)

        mono_left.out.link(depth.left)
        mono_right.out.link(depth.right)

        # YOLOv8 detection network
        try:
            detection_nn = self.pipeline.create(dai.node.YoloDetectionNetwork)
            detection_nn.setConfidenceThreshold(max(self.confidence_threshold, self.confidence))
            detection_nn.setNumClasses(self.num_classes)
            detection_nn.setCoordinateSize(self.coordinates)
            detection_nn.setIouThreshold(self.iou_threshold)
            if self.anchors:
                detection_nn.setAnchors(self.anchors)
            if self.anchor_masks:
                detection_nn.setAnchorMasks(self.anchor_masks)
            detection_nn.setBlobPath(self.blob_path)
            detection_nn.setNumInferenceThreads(2)
            detection_nn.input.setBlocking(False)
            self.use_yolo_node = True
            print("✅ Using YoloDetectionNetwork node")
        except Exception as e:
            print(f"⚠️ Unable to use YoloDetectionNetwork, falling back to NeuralNetwork: {e}")
            detection_nn = self.pipeline.create(dai.node.NeuralNetwork)
            detection_nn.setBlobPath(self.blob_path)
            detection_nn.setNumPoolFrames(4)
            detection_nn.input.setBlocking(False)
            detection_nn.setNumInferenceThreads(2)
            self.use_yolo_node = False

        cam_rgb.preview.link(detection_nn.input)

        # Output streams
        cam_out = self.pipeline.create(dai.node.XLinkOut)
        cam_out.setStreamName("rgb")
        cam_rgb.preview.link(cam_out.input)

        depth_out = self.pipeline.create(dai.node.XLinkOut)
        depth_out.setStreamName("depth")
        depth.depth.link(depth_out.input)

        detection_out = self.pipeline.create(dai.node.XLinkOut)
        detection_out.setStreamName("detections")
        detection_nn.out.link(detection_out.input)

    def detect(self):
        """
        Run one detection pass, returning:
          - result: {"predictions": [...]}
          - annotated_frame: frame with drawn boxes (for debugging)
          - raw_frame: original RGB frame
          - depth: depth frame as numpy array
        """
        # 1. Grab RGB frame
        in_rgb = self.q_rgb.get()
        frame = in_rgb.getCvFrame()
        if self.image_pub is not None:
            ros_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(ros_img)

        raw_frame = frame.copy()

        # 2. Grab depth frame
        in_depth = self.q_depth.get()
        depth = in_depth.getFrame()

        # 3. Retrieve detection results
        predictions = []
        in_det = self.q_det.tryGet()
        if in_det is not None:
            if self.use_yolo_node:
                detections = in_det.detections
                predictions = self._process_yolo_detections(detections, frame.shape[:2])
            else:
                output_tensor = in_det.getFirstLayerFp16()
                predictions = self._decode_yolo_output(output_tensor, frame.shape[:2])

        # 4. (Optional) Draw boxes on the frame for debugging
        annotated_frame = self._draw_predictions(frame, predictions)

        # 5. Add depth info to predictions
        self._add_depth_info(predictions, depth)

        result = {"predictions": predictions}
        return result, annotated_frame, raw_frame, depth

    def _process_yolo_detections(self, detections, img_shape):
        """Process YoloDetectionNetwork output and filter StopSigns"""
        predictions = []
        img_h, img_w = img_shape
        for det in detections:
            if det.confidence > self.confidence and det.label == self.stop_sign_class_id:
                x_center = det.xmin + (det.xmax - det.xmin) / 2
                y_center = det.ymin + (det.ymax - det.ymin) / 2
                w = det.xmax - det.xmin
                h = det.ymax - det.ymin
                x_pixel = x_center * img_w
                y_pixel = y_center * img_h
                w_pixel = w * img_w
                h_pixel = h * img_h
                pred = StopSignPrediction(
                    x=x_pixel,
                    y=y_pixel,
                    width=w_pixel,
                    height=h_pixel,
                    confidence=det.confidence,
                    class_name="stop_sign"
                )
                predictions.append(pred)
        return predictions

    def _decode_yolo_output(self, output, img_shape):
        """Manually decode YOLO output when using NeuralNetwork node"""
        predictions = []
        try:
            arr = np.array(output)
            img_h, img_w = img_shape
            if arr.ndim == 1 and arr.size % 84 == 0:
                num_det = arr.size // 84
                reshaped = arr.reshape(84, num_det).T
                for det in reshaped:
                    x, y, w, h = det[:4]
                    class_scores = det[4:]
                    max_score = np.max(class_scores)
                    max_id = np.argmax(class_scores)
                    if max_id == self.stop_sign_class_id and max_score > self.confidence:
                        if x <= 1.0 and y <= 1.0:
                            x_pixel = x * img_w
                            y_pixel = y * img_h
                            w_pixel = w * img_w
                            h_pixel = h * img_h
                        else:
                            x_pixel, y_pixel, w_pixel, h_pixel = x, y, w, h
                        if (10 <= w_pixel <= img_w * 0.9 and
                            10 <= h_pixel <= img_h * 0.9 and
                            0 <= x_pixel <= img_w and
                            0 <= y_pixel <= img_h):
                            pred = StopSignPrediction(
                                x=x_pixel,
                                y=y_pixel,
                                width=w_pixel,
                                height=h_pixel,
                                confidence=max_score,
                                class_name="stop_sign"
                            )
                            predictions.append(pred)
        except Exception as e:
            print(f"❌ Manual decoding failed: {e}")
        return predictions

    def _draw_predictions(self, frame, predictions):
        """Draw detection boxes on the frame (debugging only)"""
        annotated = frame.copy()
        for p in predictions:
            x1 = int(p.x - p.width / 2)
            y1 = int(p.y - p.height / 2)
            x2 = int(p.x + p.width / 2)
            y2 = int(p.y + p.height / 2)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{p.class_name}: {p.confidence:.0%}"
            cv2.putText(annotated, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return annotated

    def _add_depth_info(self, predictions, depth):
        """Add depth information (in meters) to each prediction"""
        for p in predictions:
            cx = int(p.x)
            cy = int(p.y)
            if 0 <= cx < depth.shape[1] and 0 <= cy < depth.shape[0]:
                dv = depth[cy, cx]
                p.depth = dv / 1000.0 if dv > 0 else None
            else:
                p.depth = None


class StopSignPrediction:
    """Holds a single StopSign detection result"""

    def __init__(self, x, y, width, height, confidence, class_name):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.confidence = confidence
        self.class_name = class_name
        self.depth = None

    def json(self):
        data = {
            'x': self.x,
            'y': self.y,
            'width': self.width,
            'height': self.height,
            'confidence': self.confidence,
            'class': self.class_name
        }
        if self.depth is not None:
            data['depth'] = self.depth
        return data


class StopDetectorNode(Node):
    """ROS 2 node that runs LocalOAK detection and publishes stop/no-stop signal"""

    def __init__(self):
        super().__init__('stop_detector')
        # Publisher for the stop/no-stop signal
        self.pub = self.create_publisher(Twist, '/stop_cmd_twist', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        # Timer at 10 Hz for running detection
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Assume model files are in the same directory as this script
        script_dir = os.path.dirname(__file__)
        blob_path = os.path.join(script_dir, 'yolov8n_openvino_2022.1_6shave.blob')
        config_path = os.path.join(script_dir, 'yolov8n.json')

        if not os.path.exists(blob_path) or not os.path.exists(config_path):
            self.get_logger().error(
                f"Cannot find model or config files. Check:\n"
                f"{blob_path}\n    {config_path}"
            )
            return

        # Initialize the LocalOAK detector
        self.rf = LocalOAK(
            blob_path=blob_path,
            config_path=config_path,
            confidence=0.9,
            overlap=0.5,
            rgb=True,
            depth=True
        )
        self.rf.image_pub = self.image_pub
        self.rf.bridge = self.bridge        
        self.get_logger().info("StopSign detection node started (ROS 2)")

        self.frame_count = 0
        self.prev_time = time.time()

        self.stop_counter = 0
        self.no_stop_counter = 0
        self.stop_threshold = 3        # 3continuous StopSign
        self.no_stop_threshold = 5     # 5 continuous no StopSign
        self.is_stopping = False       

    def timer_callback(self):
        # Run one detection pass
        result, annotated_frame, raw_frame, depth = self.rf.detect()
        predictions = result["predictions"]

        # Calculate and log FPS
        now = time.time()
        dt = now - self.prev_time
        fps = 1.0 / dt if dt > 0 else 0.0
        self.prev_time = now
        self.get_logger().info(f"Frame {self.frame_count}, FPS={fps:.1f}, Detected={len(predictions)}")

        twist_msg = Twist()

        if len(predictions) > 0:
            self.stop_counter += 1
            self.no_stop_counter = 0
        else:
            self.no_stop_counter += 1
            self.stop_counter = 0

        if not self.is_stopping and self.stop_counter >= self.stop_threshold:
            self.is_stopping = True
            self.get_logger().warn("▶ StopSign detected (debounced), switching to STOPPING")

        elif self.is_stopping and self.no_stop_counter >= self.no_stop_threshold:
            self.is_stopping = False
            self.get_logger().info("▶ StopSign no longer detected (debounced), switching to GO")

        if self.is_stopping:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        else:
            twist_msg.linear.x = -1.0
            twist_msg.angular.z = 0.0

        self.pub.publish(twist_msg)
        self.frame_count += 1



def main(args=None):
    rclpy.init(args=args)
    node = StopDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
