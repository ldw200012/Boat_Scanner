#!/usr/bin/python3

# Python node for object detection.
# Customized to indicate the detection of 'boat' class

import os
import rospkg
import rospy
import numpy as np
import importlib
import torch

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from damo.base_models.core.ops import RepConv
from damo.detectors.detector import build_local_model
from damo.utils.demo_utils import transform_img
from damo.structures.image_list import ImageList

class DetectionNode():

    def __init__(self):

        self.dot_cnt = 0
        self.dot_cnt_max = 5
        self.use_cuda = True
        self.device = 'cuda' if torch.cuda.is_available() and self.use_cuda else 'cpu'

        ### Customizables
        self.cam_fov = 0.57*2 # -0.57 ~ 0.57
        self.bbox_margin = 0 # pixels
        self.boat_close_enough_ratio = 0.1 # pixel width ratio
        self.conf = 0.5
        self.infer_size = [320, 320]
        self.backbone = "damoyolo_tinynasL35_M"

        # Find the path to your ROS package
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('vessel_pointcloud_scanner')  # replace with your package name
        self.ckpt_path = os.path.join(package_path, 'scripts/weights/' + self.backbone + ".pth")
        self.config = importlib.import_module("damo_configs." + self.backbone).Config()

        if "class_names" in self.config.dataset:
            self.class_names = self.config.dataset.class_names
        else:
            self.class_names = []
            for i in range(self.config.model.head.num_classes):
                self.class_names.append(str(i))
            self.class_names = tuple(self.class_names)

        self.config.dataset.size_divisibility = 0
        self.model = self._build_engine(self.config)

        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber('/camera1/image', Image, callback = self.img_cb, queue_size = 1, callback_args=0)
        self.fov_pub = rospy.Publisher('/vessel_target/cam_fov', Float32MultiArray, queue_size=1)

    def _pad_image(self, img, target_size):

        n, c, h, w = img.shape
        assert n == 1
        assert h<=target_size[0] and w<=target_size[1]
        target_size = [n, c, target_size[0], target_size[1]]
        pad_imgs = torch.zeros(*target_size)
        pad_imgs[:, :c, :h, :w].copy_(img)

        img_sizes = [img.shape[-2:]]
        pad_sizes = [pad_imgs.shape[-2:]]

        return ImageList(pad_imgs, img_sizes, pad_sizes)

    def _build_engine(self, config):

        model = build_local_model(config, self.device)
        ckpt = torch.load(self.ckpt_path, map_location=self.device)
        model.load_state_dict(ckpt['model'], strict=True)

        for layer in model.modules():
            if isinstance(layer, RepConv):
                layer.switch_to_deploy()

        model.eval()

        dumy_input = torch.ones((1, 3, self.infer_size[0], self.infer_size[1])).cuda(self.device)
        _ = model(dumy_input)

        return model

    def preprocess(self, origin_img):

        img = transform_img(origin_img, 0,
                            **self.config.test.augment.transform,
                            infer_size=self.infer_size)
                            
        # img is a image_list
        oh, ow, _  = origin_img.shape
        img = self._pad_image(img.tensors, self.infer_size)
        img = img.to(self.device)

        return img, (ow, oh)

    def postprocess(self, preds, origin_shape=None):
        
        output = preds[0].resize(origin_shape)
        bboxes = output.bbox
        scores = output.get_field('scores')
        cls_inds = output.get_field('labels')

        return bboxes,  scores, cls_inds

    def forward(self, origin_image):

        image, origin_shape = self.preprocess(origin_image)
        output = self.infer(image)
        bboxes, scores, cls_inds = self.postprocess(output, origin_shape=origin_shape)

        return bboxes, scores, cls_inds

    def infer(self, image):
        with torch.no_grad():
            output = self.model(image)
        return output
    
    def boat_detection_availability(self, bboxes, scores, cls_inds):
        available_bboxes = []
        available_scores = []
        for i in range(len(bboxes)):
            if int(cls_inds[i]) != 8: # 8 == boat
                continue
            else:
                if scores[i] > self.conf:
                    available_bboxes.append(bboxes[i])
                    available_scores.append(scores[i])
                else:
                    continue

        if len(available_bboxes) > 0:
            closest_d = float('-inf')
            closest_i = 0
            for i, box in enumerate(available_bboxes):
                y1 = int(box[3])

                if y1 > closest_d:
                    closest_d = y1
                    closest_i = i

            return available_bboxes[closest_i], available_scores[closest_i]
        else:
            return False
        
    def is_really_boat(self, img, bbox):
        img_height, img_width, channel = img.shape
        bbox_length = bbox[2] - bbox[0]
        if bbox_length >= img_width*self.boat_close_enough_ratio:
            if (bbox[0] > self.bbox_margin) and (bbox[2] < img_width-self.bbox_margin):
                return bbox[0], bbox[2]

        return False, False

    def img_cb(self, data, args):
        max_length = len("[vessel_pointcloud_scanner] ==> [detection.py] node \033[91mRUNNING\033[0m .....")
        print(" " * max_length, end="\r")
        print(f"[vessel_pointcloud_scanner] ==> [detection.py] node \033[91mRUNNING\033[0m " + "." * self.dot_cnt, end="\r", flush=True)
        self.dot_cnt = (self.dot_cnt+1) if self.dot_cnt<self.dot_cnt_max else 0

        origin_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        bboxes, scores, cls_inds = self.forward(origin_img)

        cam_left_theta = -0.0
        cam_right_theta = 0.0
        if len(bboxes) > 0:
            result = self.boat_detection_availability(bboxes, scores, cls_inds)

            if result:
                target_box, target_score = result
                img_height, img_width, channel = origin_img.shape
                bbox_length = target_box[2] - target_box[0]

                if bbox_length >= img_width*self.boat_close_enough_ratio:
                    if (target_box[0] > self.bbox_margin) and (target_box[2] < img_width-self.bbox_margin):
                        cam_left_theta = (target_box[0]-(img_width//2))*self.cam_fov/img_width
                        cam_right_theta = (target_box[2]-(img_width//2))*self.cam_fov/img_width
        
        fov_data = Float32MultiArray()
        fov_data.data.append(cam_left_theta)
        fov_data.data.append(cam_right_theta)
        self.fov_pub.publish(fov_data)

if __name__ == '__main__':
    rospy.init_node('detection')
    # rospy.loginfo("[vessel_pointcloud_scanner] ==> [detection.py] node ON")

    try:
        detection_node = DetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass