
import cv2

import argparse
import os
import platform
import sys
from pathlib import Path

import torch
from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode

class JoyrideObjectClassifier():
    def __init__(self):
        # self.weights = 'yolov5s.pt'
        # self.dnn = False
        # self.data = 'data/coco128.yaml'
        # self.fp16 = False
        # self.image_size = (640, 480)
        # self.device = select_device('') # CUDA device 0, 1, 2, 3, or CPU
        # self.model = DetectMultiBackend(self.weights, device=self.device, dnn=self.dnn, data=self.data, fp16=self.fp16)
        # self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        # self.image_size = check_img_size(self.image_size, s=self.stride)
        pass

    def detectFromImage(self, cv2_image):
        pass