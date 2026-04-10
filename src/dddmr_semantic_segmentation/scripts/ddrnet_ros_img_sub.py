#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np 
import time

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from PIL import Image as PILImage
import torch
from torchvision import transforms
from trt_interface import TensorRTInference
import pycuda.driver as cuda

class DDRNetROS(Node):

    def __init__(self, trt_inference):

        super().__init__('ddrnet_ros')

        # Check if a GPU is available and set the device accordingly
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")
        
        #set to False to save cpu up tp 20% and reach more than 15 fps
        self.declare_parameter('publish_colored_mask_result', True)
        self.publish_colored_mask_result = self.get_parameter('publish_colored_mask_result').get_parameter_value().bool_value

        self.declare_parameter('use_class_based_mask_result', False)
        self.use_class_based_mask_result = self.get_parameter('use_class_based_mask_result').get_parameter_value().bool_value

        self.trt_inference = trt_inference
        self.input_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize([.485, .456, .406], [.229, .224, .225]),
        ])

        # ROS
        best_effort_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5
        )

        cg_img_sub_loop = MutuallyExclusiveCallbackGroup()
        cg_inference_loop = MutuallyExclusiveCallbackGroup()
        self.got_frame = False
        self.br = CvBridge()

        self.img_sub = self.create_subscription(Image, 'camera/camera/color/image_raw', self.imgCb, qos_profile=best_effort_qos_profile, callback_group=cg_img_sub_loop)
        self.inferenced_result_pub = self.create_publisher(Image, 'ddrnet_inferenced_and_masked_result', 1)
        self.inferenced_mask_pub = self.create_publisher(Image, 'ddrnet_inferenced_mask', 1)
        #self.ddrnet_perspective_occupancy_grid_pub = self.create_publisher(Image, 'ddrnet/perspective_occupancy_grid', 1)

        self.inference_loop = self.create_timer(0.1, self.inferenceLoop, callback_group=cg_inference_loop)
        
    def imgCb(self, msg):

        self.frame = self.br.imgmsg_to_cv2(msg)
        self.msg_header = msg.header
        self.cropped_frame = self.frame[28:452, 0:848]
        self.got_frame = True

    def inferenceLoop(self):
        
        if(not self.got_frame):
            return 
        
        #img = PILImage.open(get_package_share_directory('dddmr_semantic_segmentation')+"/data/people.png").convert('RGB')
        #img = img.resize((848,424))
        m_stamp = self.msg_header
        nparr_cropped_frame = np.array(self.cropped_frame)
        nparr_cropped_transformed_frame = self.input_transform(nparr_cropped_frame)
        output_data = self.trt_inference.infer(nparr_cropped_transformed_frame)
        reshaped_output_data = np.reshape(output_data, (1, 19, 424, 848))
        tensor_on_device = torch.from_numpy(reshaped_output_data).float().to(self.device)
        argmax_indices = torch.argmax(tensor_on_device, dim=1)
        pred = argmax_indices.cpu().data.numpy()
        #pred = np.argmax(reshaped_output_data, 1)
        self.get_logger().debug(f"Shape of pred: {pred.shape}")
        pred_2d = pred[0,:,:].astype(np.uint8)
        
        if(self.publish_colored_mask_result):
            color_masked_inferenced_frame = self.trt_inference.getColorMask(nparr_cropped_frame, pred_2d, 0.4)
            cv2_inferenced_result = cv2.cvtColor(color_masked_inferenced_frame, cv2.COLOR_BGR2RGB)
            inferenced_result = self.br.cv2_to_imgmsg(cv2_inferenced_result)
            inferenced_result.header = m_stamp
            self.inferenced_result_pub.publish(inferenced_result)
        
        if(self.use_class_based_mask_result):
            gray_ros_image = self.br.cv2_to_imgmsg(pred_2d, encoding="mono8")
            gray_ros_image.header = m_stamp
            self.inferenced_mask_pub.publish(gray_ros_image)
        else:
            color_masked_inferenced_frame = self.trt_inference.getColor(nparr_cropped_frame, pred_2d)
            cv2_inferenced_result = cv2.cvtColor(color_masked_inferenced_frame, cv2.COLOR_BGR2RGB)
            inferenced_result = self.br.cv2_to_imgmsg(cv2_inferenced_result)
            inferenced_result.header = m_stamp
            self.inferenced_mask_pub.publish(inferenced_result)            

def main(args=None):

    rclpy.init(args=args)
    
    pkg_path = get_package_share_directory('dddmr_semantic_segmentation')
    engine_path = pkg_path + "/model/ddrnet_23_slim_dualresnet_citys_best_model_424x848.trt"
    color_csv_path = pkg_path + "/data/colors_mapillary.csv"
    trt_inference = TensorRTInference(engine_path, color_csv_path)

    DDRNROS = DDRNetROS(trt_inference)

    executor = MultiThreadedExecutor()
    executor.add_node(DDRNROS)
    executor.spin()

if __name__ == '__main__':
    main()